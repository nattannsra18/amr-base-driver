#!/usr/bin/env bash
# Install the physical localization and Nav2 stack as an ODROID boot service.
set -Eeuo pipefail

readonly SERVICE_NAME="indoor-delivery-robot-navigation.service"
readonly CONFIG_DIR="/etc/indoor-delivery-robot"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="/home/odroid/amr_ws"
MAP_FILE=""
ENABLE_MOTORS=false
ROS_DISTRO_NAME="jazzy"
DRY_RUN=false
NO_START=false

usage() {
  cat <<'EOF'
Install the physical AMCL and Nav2 stack as a systemd service.

Usage:
  sudo ./scripts/install_navigation_service.sh \
    [--map-yaml /home/odroid/amr_ws/maps/validated-map.yaml] \
    [--workspace /home/odroid/amr_ws] [--enable-motors] \
    [--ros-distro jazzy] [--no-start] [--dry-run]

The navigation service always starts the base driver, YDLidar, and scan
resampler. Map server, AMCL, and Nav2 start only after Map Management has
selected a validated active map. Supplying --map-yaml sets that initial active
map; omitting it creates a safe no-map startup state.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
info() { printf '==> %s\n' "$*"; }
run() {
  if "$DRY_RUN"; then
    printf '[dry-run]'; printf ' %q' "$@"; printf '\n'; return 0
  fi
  "$@"
}

while (($#)); do
  case "$1" in
    --workspace) (($# >= 2)) || fail "--workspace requires a path"; WORKSPACE="$2"; shift 2 ;;
    --map-yaml) (($# >= 2)) || fail "--map-yaml requires a path"; MAP_FILE="$2"; shift 2 ;;
    --enable-motors) ENABLE_MOTORS=true; shift ;;
    --ros-distro) (($# >= 2)) || fail "--ros-distro requires a name"; ROS_DISTRO_NAME="$2"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    --no-start) NO_START=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown option: $1" ;;
  esac
done

[[ "$ROS_DISTRO_NAME" =~ ^[a-z0-9_]+$ ]] || fail "Invalid ROS distribution name"
[[ -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]] || fail "ROS setup not found: /opt/ros/$ROS_DISTRO_NAME/setup.bash"
[[ -f "$WORKSPACE/install/setup.bash" ]] || fail "Workspace is not built: $WORKSPACE/install/setup.bash"
[[ -z "$MAP_FILE" || -f "$MAP_FILE" ]] || fail "Map YAML was not found: $MAP_FILE"
[[ -f "$REPOSITORY_ROOT/deploy/systemd/$SERVICE_NAME" ]] || fail "Service template was not found"
[[ "$EUID" -eq 0 || "$DRY_RUN" == true ]] || fail "Run this installer with sudo, or use --dry-run"

WORKSPACE="$(readlink -f -- "$WORKSPACE")"
SOURCE_COMMIT="$(git -C "$REPOSITORY_ROOT" rev-parse --verify HEAD 2>/dev/null || printf 'unknown')"
if [[ -n "$MAP_FILE" ]]; then
  MAP_FILE="$(readlink -f -- "$MAP_FILE")"
fi
ENV_FILE="$(mktemp)"
cleanup() { rm -f -- "$ENV_FILE"; }
trap cleanup EXIT

{
  printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO_NAME"
  printf 'AMR_WORKSPACE=%s\n' "$WORKSPACE"
  printf 'AMR_ACTIVE_MAP_LINK=%s/maps/.active-map.yaml\n' "$WORKSPACE"
  printf 'AMR_ENABLE_MOTORS=%s\n' "$ENABLE_MOTORS"
  printf 'AMR_SOURCE_COMMIT=%s\n' "$SOURCE_COMMIT"
  # Keep every robot-side ROS process on the same transport. The Agent runs as
  # a separate service account, so UDP avoids Fast DDS shared-memory permission
  # boundaries while preserving lifecycle service discovery.
  # Every ROS participant for this prototype runs on the ODROID. Keep DDS on
  # loopback; the Robot Agent's WebSocket remains network-accessible.
  printf 'ROS_DOMAIN_ID=0\nROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST\nRMW_IMPLEMENTATION=rmw_fastrtps_cpp\nFASTDDS_BUILTIN_TRANSPORTS=UDPv4\n'
} > "$ENV_FILE"

info "Installing the navigation boot service"
run install -d -m 0755 "$CONFIG_DIR"
run install -o root -g root -m 0644 "$ENV_FILE" "$CONFIG_DIR/navigation.env"
run install -o root -g root -m 0755 \
  "$REPOSITORY_ROOT/deploy/indoor-delivery-robot-navigation" \
  "/usr/local/sbin/indoor-delivery-robot-navigation"
run install -o root -g root -m 0755 \
  "$REPOSITORY_ROOT/deploy/indoor-delivery-robot-control" \
  "/usr/local/sbin/indoor-delivery-robot-control"
run install -o root -g root -m 0440 \
  "$REPOSITORY_ROOT/deploy/sudoers.d/indoor-delivery-robot-control" \
  "/etc/sudoers.d/indoor-delivery-robot-control"
run install -o root -g root -m 0644 \
  "$REPOSITORY_ROOT/deploy/systemd/$SERVICE_NAME" \
  "/etc/systemd/system/$SERVICE_NAME"
run systemctl daemon-reload

# navigation.launch.py owns the LiDAR driver. A separate legacy unit would
# create a second driver against the same serial device.
if systemctl list-unit-files --type=service | grep -q '^ydlidar-x3.service'; then
  run systemctl disable --now ydlidar-x3.service
fi

run systemctl enable "$SERVICE_NAME"
if [[ -n "$MAP_FILE" ]]; then
  run /usr/local/sbin/indoor-delivery-robot-control select-map "$MAP_FILE"
fi
if "$NO_START"; then
  info "Installed without starting the service (--no-start)"
else
  run systemctl restart "$SERVICE_NAME"
fi

printf '\nNavigation boot service installed.\n'
printf 'Status: sudo systemctl status %s\n' "$SERVICE_NAME"
printf 'Logs:   sudo journalctl -u %s -f\n' "$SERVICE_NAME"
printf 'AMCL still requires an operator-confirmed initial pose after the robot is placed.\n'
