# My Physical AMR Base Driver

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![Platform](https://img.shields.io/badge/SBC-ODROID--C4-4C566A)](https://www.hardkernel.com/shop/odroid-c4/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

This is the ROS 2 Jazzy package I use on my physical indoor delivery robot. It
connects an ODROID-C4, ESP32 motor controller, YDLidar X3, wheel encoders, and
MPU6050 so I can drive the robot, build maps with `slam_toolbox`, and localize
with AMCL.

This repository contains my robot-side hardware code, physical Nav2 profile,
and the Robot Agent used to connect this robot to my web control platform.
The related projects are
[Indoor Delivery Robot Platform](https://github.com/nattannsra18/indoor-delivery-robot)
and
[AMR Navigation, Vision & Diagnostics](https://github.com/nattannsra18/amr-navigation-vision-diagnostics).

> [!CAUTION]
> A fresh base launch starts with motor output disabled. After I verify
> telemetry and the physical area, I arm once for the attended session and do
> not use normal AMCL, Nav2, or web-service restarts to lock the motors. The
> physical motor cutoff stays within reach at all times.

## What I use

| Part | Role in my robot |
|---|---|
| ODROID-C4 | Ubuntu 24.04 and ROS 2 Jazzy |
| ESP32-WROOM DevKit, 38-pin | Wheel PI, encoders, MPU6050, and UART telemetry |
| TB6612FNG | Left and right TT motor driver |
| TT motors, encoders, and passive caster | Differential-drive base |
| YDLidar X3 | Scan for SLAM and localization |
| `robot_localization` | Fuses wheel odometry with IMU data |
| `slam_toolbox` | Builds a 2D map |
| AMCL + map server | Localizes the robot on a saved map |

My physical-test notes are in [docs/VALIDATION.md](docs/VALIDATION.md). I keep
room maps and rosbags out of Git because they contain private layout and test
data.

## System overview

```mermaid
flowchart LR
  user["Me<br/>RViz / keyboard"] --> command["Teleop or Nav2 command"]

  subgraph odroid["ODROID-C4 · ROS 2 Jazzy"]
    relay["cmd_vel_passthrough"]
    bridge["serial_bridge"]
    resampler["scan_resampler"]
    ekf["robot_localization EKF"]
    slam["SLAM Toolbox"]
    amcl["Map server + AMCL"]
  end

  subgraph sensors["Sensors"]
    lidar["YDLidar X3"]
    encoder["Wheel encoders"]
    imu["MPU6050"]
  end

  subgraph drive["Drive hardware"]
    esp32["ESP32<br/>wheel PI + watchdog"]
    driver["TB6612FNG"]
    motors["Left + right motors"]
  end

  command -->|"/cmd_vel"| relay
  relay -->|"/cmd_vel_safe"| bridge
  bridge <-->|"115200 baud<br/>binary + CRC-16"| esp32
  esp32 --> driver --> motors
  encoder --> esp32
  imu --> esp32
  esp32 -->|"/wheel/odometry<br/>/imu/data_raw<br/>/diagnostics"| bridge
  bridge --> ekf
  lidar -->|"/scan_raw"| resampler
  resampler -->|"/scan"| slam
  resampler -->|"/scan"| amcl
  ekf -->|"/odometry/filtered"| slam
  ekf -->|"/odometry/filtered"| amcl
```

The ODROID runs ROS, state estimation, and diagnostics. The ESP32 handles the
wheel loop and turns PWM off when commands stop arriving. SLAM and AMCL share
the same scan and EKF output, but I run them as separate modes.

## Web Robot Agent

The source for the Robot Agent is included at
[`robot_agent/amr_web_bridge`](robot_agent/amr_web_bridge). It is the ROS 2
package that authenticates this ODROID with the FastAPI control plane, reports
telemetry, diagnostics, maps and Nav2 paths, and accepts only authenticated
navigation or emergency commands. The physical ODROID profile and hardened
systemd unit are included with the package.

I build the base driver and Agent together from the same workspace:

```bash
cd ~/amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select amr_base_driver amr_web_bridge
source install/setup.bash
```

The profile in
`robot_agent/amr_web_bridge/config/profiles/odroid_c4_tt_prototype.yaml` is
the starting point for this chassis. Connection URL, enrollment token, and
credential file are deployment secrets and stay outside Git. The Agent can be
started with the profile after those values are configured:

```bash
ros2 run amr_web_bridge web_bridge_node --ros-args \
  --params-file ~/amr_ws/src/amr_base_driver/robot_agent/amr_web_bridge/config/profiles/odroid_c4_tt_prototype.yaml \
  -p server_url:=wss://control.example.com
```

For the ODROID service installation, I use the included installer. It builds a
relocatable Agent workspace, installs the profile and credentials with limited
permissions, then enables `indoor-delivery-robot-agent.service`. I check the
commands first with `--dry-run`; the enrollment token is read from a protected
file or environment variable, never from the command line.

```bash
sudo ./scripts/install_robot_agent.sh \
  --profile robot_agent/amr_web_bridge/config/profiles/odroid_c4_tt_prototype.yaml \
  --control-url wss://control.example.com \
  --enrollment-token-file /path/to/protected-token
```

## Power and pinout

I removed the wiring image because I want the published diagram to match the
assembled robot exactly. The following table is the pinout currently used by my
firmware. The controller is a 38-pin ESP32-WROOM DevKit in a 38-pin I/O
expansion board.

| Used for | ESP32 pin | Connected to |
|---|---:|---|
| UART receive from ODROID | GPIO34 (`RX2`) | ODROID-C4 pin 32, `UART_EE_C` TX |
| UART transmit to ODROID | GPIO23 (`TX2`) | ODROID-C4 pin 26, `UART_EE_C` RX |
| UART ground | GND | ODROID-C4 pin 6 GND |
| Left encoder A / B | GPIO25 / GPIO26 | Left motor encoder A / B |
| Right encoder A / B | GPIO35 / GPIO33 | Right motor encoder A / B; GPIO35 needs an external pull-up |
| MPU6050 SDA / SCL | GPIO21 / GPIO22 | MPU6050 SDA / SCL |
| Left motor PWM / direction | GPIO14 / GPIO18 / GPIO19 | TB6612FNG `PWMA` / `AIN1` / `AIN2` |
| Right motor PWM / direction | GPIO27 / GPIO16 / GPIO17 | TB6612FNG `PWMB` / `BIN1` / `BIN2` |
| Motor standby | GPIO32 | TB6612FNG `STBY`; LOW stops motor output |

GPIO1 and GPIO3 stay free for USB flashing and the Arduino serial monitor.

| Power path | What it powers |
|---|---|
| 2S 18650, 5000 mAh pack | TB6612FNG motor supply and ESP32 expansion-board DC input |
| 5 V, 5000 mAh power bank | ODROID-C4 |
| ODROID-C4 USB host | YDLidar X3 power and data |
| Shared GND | ODROID, ESP32, motor driver, and sensors use the same signal reference |

The 2S positive rail and 5 V power-bank rail are separate. I never apply the
2S positive rail to the ODROID, GPIO pins, or a logic rail. Before powering
the robot, I check battery polarity, the fuse/cutoff, carrier input range,
logic-voltage jumper, and ground continuity.

## Command path and safety

I use direct passthrough as the normal command path. The old Drive Supervisor
stays in the repository for experiments, but I keep it disabled because it
made SLAM worse on my physical robot.

```mermaid
flowchart LR
  command["/cmd_vel"] --> fresh{"Command is fresh?"}
  fresh -- No --> stop["STOP"]
  fresh -- Yes --> relay["cmd_vel_passthrough"]
  relay --> safe["/cmd_vel_safe"]
  safe --> host{"Host checks"}
  host -- "locked / stale telemetry / wheel fault" --> stop
  host -- Pass --> serial["UART wheel targets"]
  serial --> mcu{"ESP32 checks"}
  mcu -- "bad packet / watchdog / encoder stall" --> pwm0["PWM = 0<br/>fault latched"]
  mcu -- Pass --> pi["Wheel PI"]
  pi --> motors["TB6612FNG + motors"]
  stop --> serialstop["UART STOP"]
  serialstop --> pwm0
```

I rely on several layers of protection:

1. A cold base launch defaults to `enable_motors:=false`; I arm after the
   physical area and telemetry are ready.
2. Teleop has a dead-man timeout.
3. The host motion guard compares the command with wheel feedback.
4. The serial bridge refuses to arm with stale telemetry or an active fault.
5. The ESP32 checks packets and stops after a 300 ms command timeout.
6. The physical motor cutoff stays within reach during every attended test.

A software stop is useful, but it is not a replacement for a certified
emergency stop.

## Frames, odometry, and geometry

```mermaid
flowchart TB
  map["map"] -->|"SLAM Toolbox or AMCL<br/>one authority at a time"| odom["odom"]
  odom -->|"robot_localization EKF"| footprint["base_footprint"]
  footprint -->|"static z = 0.0325 m"| base["base_link"]
  base -->|static| laser["laser_frame"]
  base -->|static| imu["imu_link"]
  wheel["Wheel velocity + yaw rate"] --> ekf["EKF"] --> odom
  gyro["MPU6050 angular velocity"] --> ekf
  scan["YDLidar /scan"] --> matching["LiDAR map matching"] --> map
```

The MPU6050 has no magnetometer. I use wheel data and gyro yaw rate for local
motion, then use LiDAR mapping or localization to correct long-term heading.
Only one node should publish each TF edge.

| Item | Measurement | Transform from `base_link` |
|---|---:|---:|
| Wheel radius | 0.0325 m | n/a |
| Wheel separation | 0.355 m centre-to-centre | n/a |
| LiDAR | x=0.345 m from front, z=0.395 m from floor | x=-0.042, y=-0.005, z=0.3625, yaw=0 |
| MPU6050 | x=0.030 m from front, z=0.075 m from floor | x=0.273, y=0, z=0.0425, yaw=0 |

I use this conservative footprint for physical navigation:

```text
[[ 0.303,  0.190],
 [ 0.303, -0.190],
 [-0.087, -0.190],
 [-0.087,  0.190]]
```

I remeasure this geometry after changing wheels, hubs, sensor mounts, or the
caster.

## ROS interfaces

| Topic | What I use it for |
|---|---|
| `/scan_raw` | Original YDLidar samples |
| `/scan` | Angle-aware, fixed 360-beam scan |
| `/wheel/odometry` | Encoder-derived base motion |
| `/odometry/filtered` | EKF output for SLAM and localization |
| `/imu/data_raw` | Calibrated MPU6050 measurement |
| `/joint_states` | Wheel state |
| `/diagnostics` | Serial, MCU, motion, and sensor health |

| Interface | What I use it for |
|---|---|
| `/cmd_vel` | Input velocity command |
| `/cmd_vel_safe` | The only command topic sent to the base bridge |
| `/set_motors_enabled` | Arm or lock the motors without restarting ROS |
| `/clear_motor_fault` | Clear a latched fault after I fix its physical cause |
| `/reset_wheel_odometry` | Reset integrated wheel pose while stopped |

## Build and first checks

I need Ubuntu 24.04 with ROS 2 Jazzy, `robot_localization`, `slam_toolbox`,
Nav2 map server, AMCL, and `ydlidar_ros2_driver` in the same colcon workspace.
My USB rules map `/dev/robot-esp32` to port 1.4 and `/dev/robot-lidar` to port
1.1. The production UART link uses `/dev/ttyAML6` at 115200 baud.

```bash
cd ~/amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select amr_base_driver
source install/setup.bash
colcon test --packages-select amr_base_driver
colcon test-result --verbose
```

I start a new base session with motor output disabled, verify the system, then
arm once for the attended run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_base_driver base_hardware.launch.py enable_motors:=false
```

Before a floor test, I check `/diagnostics`, `/wheel/odometry`,
`/imu/data_raw`, `/scan`, and the TF chain. In a clear area with the cutoff
ready, I arm without restarting ROS:

```bash
ros2 service call /set_motors_enabled std_srvs/srv/SetBool "{data: true}"
```

I use the physical cutoff or a deliberate software stop when motion must stop;
normal localization, Nav2, and web-service restarts do not lock the motors.

## Mapping and localization

The X3 publishes original points on `/scan_raw`. `scan_resampler` maps them by
angle into a fixed 360-beam `/scan`. I keep the YDLidar SDK `fixed_resolution`
option disabled because it can truncate points while the LiDAR motor settles.

To map, I start the nodes first, then arm after checking telemetry and the
clear travel area:

```bash
ros2 launch amr_base_driver mapping.launch.py enable_motors:=false
```

For attended mapping, I first check the fuse, cutoff, diagnostics, and clear
floor. Smooth forward segments and rolling arcs make better maps than fast
pivots or obstacle contact. When I finish, I save the map from another
terminal:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/indoor_map
```

To localize on a saved map:

```bash
ros2 launch amr_base_driver localization.launch.py \
  map_yaml:=/absolute/path/to/indoor_map.yaml \
  enable_motors:=false
```

In RViz, I set the initial pose in the `map` frame and make sure `/scan`
overlaps the map. If another launch already publishes
`base_link -> laser_frame`, I use `publish_sensor_tf:=false` until I remove
the duplicate TF publisher.

## Nav2 status

I am validating the physical Nav2 profile in my working tree. It uses AMCL,
Navfn, Regulated Pure Pursuit, velocity smoothing, and a laser collision
monitor with conservative limits for the passive caster. I will publish its
launch file and repeatable floor-test procedure after the remaining obstacle
test is complete.

## Keyboard station and fault recovery

`robot_station.sh station` opens the mapping stack, a WASD controller, and a
live sensor dashboard. It publishes at 20 Hz while I hold a movement key and
sends zero after 0.35 seconds without input. Space or X stops, Q exits, and R
resets wheel odometry while stopped.

```bash
./robot_station.sh station
./robot_station.sh stop
```

After I physically clear the robot and confirm it has stopped, I can clear a
latched fault:

```bash
ros2 service call /clear_motor_fault std_srvs/srv/Trigger '{}'
```

I do not move again until `/diagnostics` reports `mcu_fault=0`, an empty
`host_motion_fault`, fresh serial telemetry, and zero wheel motion.

## Firmware and UART

The ESP32 source and flashing guide are in
[`firmware/esp32`](firmware/esp32/README.md). It uses a fixed 44-byte binary
telemetry frame with CRC-16. I use a 40 MHz flash clock because the physical
controller produced repeatable bootloader checksum failures at 80 MHz.

| ODROID-C4 | ESP32 |
|---|---|
| Pin 32 TX | GPIO34 RX2 |
| Pin 26 RX | GPIO23 TX2 |
| Ground | Ground |

The ESP32 has its own power source. I connect its USB cable only while
flashing.

## Current limitations

- I keep Drive Supervisor disabled for normal use because it previously
  distorted SLAM on this robot.
- The MPU6050 cannot provide an absolute heading by itself.
- Encoders cannot tell me if a wheel hub slips on its motor shaft.
- Floor friction and caster orientation still affect low-speed movement.
- I repeat geometry, stopping, and localization tests after physical changes.
- Software safeguards do not replace a physical emergency-stop circuit.

## Repository layout

```text
amr-base-driver/
├── amr_base_driver/       # ROS nodes, protocol, safety, and conditioning
├── config/                # Base, EKF, LiDAR, SLAM, and AMCL parameters
├── launch/                # Hardware, mapping, and localization bringup
├── firmware/esp32/        # ESP32 source and flashing guide
├── test/                  # Protocol, safety, estimation, and scan tests
├── docs/VALIDATION.md     # Physical-test notes
├── 99-amr-serial.rules    # Stable USB device naming
└── robot_station.sh       # Mapping/calibration workstation
```

## License

This project is licensed under the [Apache License 2.0](LICENSE).

## Author

Built by [nattannsra18](https://github.com/nattannsra18).
