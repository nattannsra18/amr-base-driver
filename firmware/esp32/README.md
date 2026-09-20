# ESP32 base firmware

The deployed source is in
`robot_base_tb6612_mapping_v3_floor_start/`. It targets an ESP32-WROOM DevKit
and controls two TB6612FNG motor channels, two quadrature encoders, an MPU6050,
and the binary UART telemetry link consumed by `amr_base_driver`.

## Safety behavior

- PWM is zero at boot.
- The host must refresh wheel commands within 300 ms.
- Invalid commands are rejected.
- IMU, encoder-stall, and encoder-direction faults stop both motors and latch.
- `CLEAR` only releases a fault while the requested wheel speeds are zero.
- PWM, acceleration, and commanded RPM are bounded.

Do not remove or bypass these guards during ordinary testing. A successful
fixed-PWM bench sketch does not validate the closed-loop firmware under caster
and floor load.

## Reproducible build

The last verified toolchain was:

- Arduino CLI 1.5.1
- ESP32 Arduino core 3.3.11
- FQBN `esp32:esp32:esp32`

Compile without writing artifacts into the source tree:

```bash
arduino-cli core install esp32:esp32@3.3.11
build_dir="$(mktemp -d)"
arduino-cli compile \
  --fqbn esp32:esp32:esp32 \
  --build-path "$build_dir" \
  firmware/esp32/robot_base_tb6612_mapping_v3_floor_start
```

The 2026-09-20 verification used 317,963 bytes (24%) of flash and 24,640 bytes
(7%) of dynamic memory.

## Flashing on the ODROID-C4

Flashing is intentionally separate from normal build and ROS testing. First:

1. Stop every ROS process that may own the ESP32 serial port.
2. Raise or mechanically isolate the driven wheels and switch motor power off.
3. Identify the ESP32 and LiDAR ports again; never assume USB numbering.
4. Keep the physical motor cutoff within reach.

The ODROID system `esptool` used for this robot lacks its downloadable stub,
so use ROM bootloader mode and verify the chip before writing:

```bash
python3 -m esptool \
  --chip esp32 --port /dev/ttyUSBX --baud 115200 --no-stub chip_id
```

After confirming that `/dev/ttyUSBX` is the authorized ESP32, flash the four
images created by Arduino CLI:

```bash
python3 -m esptool \
  --chip esp32 --port /dev/ttyUSBX --baud 115200 --no-stub \
  write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x1000  robot_base_tb6612_mapping_v3_floor_start.ino.bootloader.bin \
  0x8000  robot_base_tb6612_mapping_v3_floor_start.ino.partitions.bin \
  0xe000  boot_app0.bin \
  0x10000 robot_base_tb6612_mapping_v3_floor_start.ino.bin
```

Do not enable motors after flashing until live ROS diagnostics show fresh
telemetry, `mcu_fault=0`, an empty host-motion fault, and `parse_errors=0`.
Validate IMU, encoder, odometry, and watchdog behavior with motor power locked
before an attended floor test.
