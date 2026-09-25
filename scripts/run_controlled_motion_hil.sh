#!/usr/bin/env bash
set -euo pipefail

if [[ ${1:-} != --execute ]]; then
  echo 'Dry run only. This suite moves the robot about 0.12 m in both directions.'
  echo 'Clear the floor, keep the motor cutoff in reach, then rerun with --execute.'
  exit 2
fi

for mode in straight reverse arc_left arc_right; do
  echo "=== HIL ${mode} ==="
  if [[ ${mode} == arc_* ]]; then
    ros2 run amr_base_driver calibration_run --ros-args \
      -p mode:="${mode}" -p target_angle:=0.20 \
      -p arc_linear_speed:=0.05 -p arc_angular_speed:=0.28 \
      -p max_duration:=8.0
  else
    ros2 run amr_base_driver calibration_run --ros-args \
      -p mode:="${mode}" -p target_distance:=0.12 -p linear_speed:=0.06 \
      -p max_duration:=8.0
  fi
done

echo 'HIL PASS: forward, reverse, and both rolling-arc encoder signs agree.'
