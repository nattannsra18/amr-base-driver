#!/usr/bin/env bash
# Verify the live ROS graph required by the physical delivery-robot Agent.
set -u

mode="physical"
usage() {
  cat <<'EOF'
Usage: verify_hardware_contract.sh [--physical|--simulation]

Verify the live ROS graph against Hardware Interface Contract v1.0.
Run after sourcing ROS 2, the robot workspace, and starting the full robot
bringup. Physical mode additionally requires battery and physical E-stop state.

ROS names may be remapped with environment variables:
  ODOM_TOPIC SCAN_TOPIC MAP_TOPIC AMCL_POSE_TOPIC PATH_TOPIC
  DIAGNOSTICS_TOPIC CMD_VEL_TOPIC BATTERY_TOPIC PHYSICAL_ESTOP_TOPIC
  NAVIGATE_ACTION COMPUTE_PATH_ACTION LOAD_MAP_SERVICE AMCL_NODE
  GLOBAL_LOCALIZATION_SERVICE BASE_FRAME
EOF
}
case "${1:-}" in ""|--physical) ;; --simulation) mode="simulation" ;; -h|--help) usage; exit 0 ;; *) usage >&2; exit 2 ;; esac
if ! command -v ros2 >/dev/null 2>&1; then echo "FAIL HIC-ENV01 ros2 CLI is unavailable; source ROS 2 and the robot workspace"; exit 2; fi
odom_topic="${ODOM_TOPIC:-/odom}"; scan_topic="${SCAN_TOPIC:-/scan}"; map_topic="${MAP_TOPIC:-/map}"; amcl_pose_topic="${AMCL_POSE_TOPIC:-/amcl_pose}"; path_topic="${PATH_TOPIC:-/plan}"; diagnostics_topic="${DIAGNOSTICS_TOPIC:-/diagnostics}"; cmd_vel_topic="${CMD_VEL_TOPIC:-/cmd_vel}"; battery_topic="${BATTERY_TOPIC:-/battery_state}"; physical_estop_topic="${PHYSICAL_ESTOP_TOPIC:-/safety/physical_estop}"; navigate_action="${NAVIGATE_ACTION:-/navigate_to_pose}"; compute_path_action="${COMPUTE_PATH_ACTION:-/compute_path_to_pose}"; load_map_service="${LOAD_MAP_SERVICE:-/map_server/load_map}"; amcl_node="${AMCL_NODE:-/amcl}"; global_localization_service="${GLOBAL_LOCALIZATION_SERVICE:-/reinitialize_global_localization}"; base_frame="${BASE_FRAME:-base_footprint}"
passes=0; failures=0
pass() { passes=$((passes + 1)); echo "PASS $1 $2"; }
fail() { failures=$((failures + 1)); echo "FAIL $1 $2"; }
check_topic_type() { local actual_type; actual_type="$(ros2 topic type "$2" 2>/dev/null | head -n 1)"; [[ "$actual_type" == "$3" ]] && pass "$1" "$2 uses $3" || fail "$1" "$2 expected $3, observed ${actual_type:-missing}"; }
check_fresh_message() { timeout "$3" ros2 topic echo --once "$2" >/dev/null 2>&1 && pass "$1" "$2 produced a message within ${3}s" || fail "$1" "$2 produced no message within ${3}s"; }
check_action_type() { ros2 action list -t 2>/dev/null | grep -Fqx "$2 [$3]" && pass "$1" "$2 uses $3" || fail "$1" "$2 with $3 is unavailable"; }
check_service_type() { ros2 service list -t 2>/dev/null | grep -Fqx "$2 [$3]" && pass "$1" "$2 uses $3" || fail "$1" "$2 with $3 is unavailable"; }
check_tf() { timeout 5 ros2 run tf2_ros tf2_echo "$2" "$3" 2>/dev/null | grep -qm1 '^Translation:' && pass "$1" "TF $2 -> $3 is available" || fail "$1" "TF $2 -> $3 is unavailable"; }
check_topic_type HIC-T01 "$odom_topic" nav_msgs/msg/Odometry; check_fresh_message HIC-D01 "$odom_topic" 0.5
check_topic_type HIC-T02 "$scan_topic" sensor_msgs/msg/LaserScan; check_fresh_message HIC-D02 "$scan_topic" 1.0
check_topic_type HIC-T03 "$map_topic" nav_msgs/msg/OccupancyGrid; check_fresh_message HIC-D03 "$map_topic" 5.0
check_topic_type HIC-T04 "$amcl_pose_topic" geometry_msgs/msg/PoseWithCovarianceStamped; check_fresh_message HIC-D04 "$amcl_pose_topic" 1.0
check_topic_type HIC-T05 "$path_topic" nav_msgs/msg/Path; check_topic_type HIC-T06 "$diagnostics_topic" diagnostic_msgs/msg/DiagnosticArray; check_fresh_message HIC-D06 "$diagnostics_topic" 3.0; check_topic_type HIC-T07 "$cmd_vel_topic" geometry_msgs/msg/Twist
check_tf HIC-F01 map odom; check_tf HIC-F02 odom "$base_frame"; check_action_type HIC-A01 "$navigate_action" nav2_msgs/action/NavigateToPose; check_action_type HIC-A02 "$compute_path_action" nav2_msgs/action/ComputePathToPose; check_service_type HIC-S01 "$load_map_service" nav2_msgs/srv/LoadMap; check_service_type HIC-S03 "$global_localization_service" std_srvs/srv/Empty
amcl_state="$(ros2 lifecycle get "$amcl_node" 2>/dev/null || true)"; grep -Eqi '(^|[[:space:]])active([[:space:]]|$)' <<<"$amcl_state" && pass HIC-S02 "$amcl_node lifecycle is active" || fail HIC-S02 "$amcl_node lifecycle is not active"
if [[ "$mode" == physical ]]; then check_topic_type HIC-T08 "$battery_topic" sensor_msgs/msg/BatteryState; check_fresh_message HIC-D08 "$battery_topic" 5.0; check_topic_type HIC-T09 "$physical_estop_topic" std_msgs/msg/Bool; check_fresh_message HIC-D09 "$physical_estop_topic" 2.0; else echo "SKIP HIC-T08 battery sensor is a physical-hardware acceptance gate"; echo "SKIP HIC-T09 physical E-stop is a physical-hardware acceptance gate"; fi
echo "Hardware Interface Contract v1.0: $passes passed, $failures failed ($mode mode)"
((failures == 0)) || exit 1
