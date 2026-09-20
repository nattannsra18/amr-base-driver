# Physical robot validation

This document records the latest validated state of the physical ODROID-C4
robot. It is evidence for the current configuration, not a guarantee for a
different chassis, wheel load, floor, or LiDAR mounting position.

## Platform

- Ubuntu 24.04 and ROS 2 Jazzy on ODROID-C4
- Differential drive with TT motors and a passive caster
- ESP32 wheel control and encoder telemetry
- MPU6050 fused with wheel odometry by `robot_localization`
- YDLidar X3 at approximately 12.25 Hz
- `slam_toolbox` synchronous mapping
- Nav2 AMCL localization

## Mapping baseline

The currently validated mapping baseline uses:

- Drive Supervisor disabled
- EKF `transform_time_offset: 0.0`
- map resolution `0.03 m/pixel`
- SLAM keyframes at `0.10 s`, `0.05 m`, and `0.05 rad`
- smooth forward motion and rolling `Q/E` arcs
- short pauses after turns and no contact with obstacles

Static LiDAR checks at approximately 1 m and 2 m were within 2 percent. A
3.01 m measurement was about 2.9 percent high and was treated as less reliable
because of target angle and surrounding objects. No global scale correction is
applied.

The map and rosbag from the user's room are intentionally excluded from Git.
They contain private floor-layout information and experiment data.

## Localization comparison

The first AMCL run used Nav2 defaults (`update_min_d: 0.25`,
`update_min_a: 0.20`). The dedicated physical-robot profile changes only these
thresholds to `0.10 m` and `0.10 rad`; the motion model and noise parameters
remain at their baseline values.

| Metric | Nav2 default | Physical profile |
|---|---:|---:|
| Pose samples | 66 | 142 |
| Return-position difference | 0.129 m | 0.016 m |
| Return-yaw difference | 6.48 deg | 4.20 deg |
| Final sigma X / Y | 0.229 / 0.222 m | 0.119 / 0.106 m |
| Final sigma yaw | 17.91 deg | 7.26 deg |
| Largest pose update | 0.263 m / 14.89 deg | 0.145 m / 11.36 deg |
| Updates over 0.5 m or 45 deg | 0 | 0 |

The routes and durations were not identical, so this is an attended functional
comparison rather than a controlled scientific A/B test. The result is strong
enough to retain the `0.10/0.10` profile and repeat it after future mechanical
or sensor changes.

## Known limitations

- The Drive Supervisor is deprecated/experimental because its caster
  compensation previously distorted SLAM. Base, mapping, and localization
  default to direct relay mode. Do not enable it during normal operation.
- A loose wheel hub invalidated one mapping run even though encoder telemetry
  remained healthy. Encoders are upstream of the wheel-to-shaft interface, so
  physical witness marks and hub inspection remain necessary.
- The MPU6050 has no magnetometer. Absolute long-term yaw comes from LiDAR
  mapping/localization, not the IMU alone.
- High-rate in-place pivots increase scan motion distortion. Prefer rolling
  arcs for mapping and localization validation.

## Verification snapshot

On 2026-09-20:

- `colcon build --symlink-install --packages-select amr_base_driver` passed.
- All 34 `amr_base_driver` tests passed.
- The localization launch loaded a 342 by 223 map at 0.03 m/pixel.
- Map server and AMCL reached the active lifecycle state.
- The complete TF chain and `/amcl_pose` were available.
- No MCU, host-motion, or serial parsing fault occurred in the final run.

### Initial physical Nav2 bench validation

- The physical footprint, conservative costmaps, Navfn planner, Regulated Pure
  Pursuit controller, velocity smoother, and collision monitor loaded without
  parameter or plugin errors.
- `controller_server`, `planner_server`, `smoother_server`,
  `velocity_smoother`, `collision_monitor`, and `bt_navigator` all reached the
  active lifecycle state with motor output locked.
- Local/global costmap and published-footprint topics were available, and the
  command chain had exactly one publisher and subscriber at each stage.
- A temporary identity `map -> odom` transform was used only to exercise the
  lifecycle without asserting a real initial pose. No navigation goal was sent
  and no autonomous floor motion has been validated yet.

### First attended Nav2 floor tests

- A 0.30 m straight goal succeeded, moving 0.236 m before entering the 0.08 m
  goal tolerance. Motion was straight and stopped smoothly with no fault.
- A 90 degree rotation near an obstacle correctly aborted on predicted
  collision. Repeating in open space exposed an RPP acceleration deadlock:
  `/cmd_vel_nav` stayed at 0.04 rad/s because the controller applied its low
  angular acceleration limit against stationary measured odometry every cycle.
- The RPP internal `max_angular_accel` is therefore 6.0 rad/s^2, while the
  downstream open-loop velocity smoother remains the physical limiter at
  0.40 rad/s^2. This changes one subsystem and preserves the gradual motor
  command ramp.
