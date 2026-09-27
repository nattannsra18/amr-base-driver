#!/usr/bin/env bash
# Install a low-overhead MJPEG camera stream outside the ROS 2/DDS data path.
set -Eeuo pipefail

readonly SERVICE_NAME="indoor-delivery-robot-camera.service"
readonly CONFIG_DIR="/etc/indoor-delivery-robot"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
CAMERA_DEVICE="/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CAMERA_HOST="127.0.0.1"
CAMERA_PORT="8081"
CAMERA_RESOLUTION="352x288"
CAMERA_DYNAMIC_FRAMERATE="0"
DRY_RUN=false
NO_START=false

usage() {
  cat <<'EOF'
Install the low-latency MJPEG camera service.

Usage:
  sudo ./scripts/install_camera_stream.sh \
    [--device /dev/v4l/by-id/...-video-index0] \
    [--host 127.0.0.1] [--port 8081] [--resolution 352x288] \
    [--dynamic-framerate 0|1] \
    [--no-start] [--dry-run]

The default loopback binding keeps the raw stream private. The outbound camera
relay sends authenticated JPEG frames to the control plane over WSS.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
run() {
  if "$DRY_RUN"; then
    printf '[dry-run]'; printf ' %q' "$@"; printf '\n'
  else
    "$@"
  fi
}

while (($#)); do
  case "$1" in
    --device) (($# >= 2)) || fail "--device requires a path"; CAMERA_DEVICE="$2"; shift 2 ;;
    --host) (($# >= 2)) || fail "--host requires an address"; CAMERA_HOST="$2"; shift 2 ;;
    --port) (($# >= 2)) || fail "--port requires a number"; CAMERA_PORT="$2"; shift 2 ;;
    --resolution) (($# >= 2)) || fail "--resolution requires WIDTHxHEIGHT"; CAMERA_RESOLUTION="$2"; shift 2 ;;
    --dynamic-framerate) (($# >= 2)) || fail "--dynamic-framerate requires 0 or 1"; CAMERA_DYNAMIC_FRAMERATE="$2"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    --no-start) NO_START=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown option: $1" ;;
  esac
done

[[ "$CAMERA_DEVICE" == /dev/* ]] || fail "Camera device must be under /dev"
[[ "$CAMERA_HOST" =~ ^[0-9a-fA-F:.]+$ ]] || fail "Camera host must be an IP address"
[[ "$CAMERA_PORT" =~ ^[0-9]+$ ]] && ((CAMERA_PORT >= 1 && CAMERA_PORT <= 65535)) || fail "Invalid camera port"
[[ "$CAMERA_RESOLUTION" =~ ^[0-9]+x[0-9]+$ ]] || fail "Resolution must be WIDTHxHEIGHT"
[[ "$CAMERA_DYNAMIC_FRAMERATE" =~ ^[01]$ ]] || fail "Dynamic frame rate must be 0 or 1"
[[ -x /usr/bin/ustreamer || "$DRY_RUN" == true ]] || fail "Install ustreamer first: sudo apt-get install ustreamer"
[[ -e "$CAMERA_DEVICE" || "$DRY_RUN" == true ]] || fail "Camera capture device not found: $CAMERA_DEVICE"
[[ -f "$REPOSITORY_ROOT/deploy/systemd/$SERVICE_NAME" ]] || fail "Service template was not found"
[[ "$EUID" -eq 0 || "$DRY_RUN" == true ]] || fail "Run this installer with sudo, or use --dry-run"

if ! "$DRY_RUN" && [[ "$CAMERA_HOST" != "127.0.0.1" ]] && ! ip -brief address show | grep -Fq "$CAMERA_HOST"; then
  fail "Camera host is not assigned to this computer: $CAMERA_HOST"
fi

ENV_FILE="$(mktemp)"
cleanup() { rm -f -- "$ENV_FILE"; }
trap cleanup EXIT
{
  printf 'CAMERA_DEVICE=%s\n' "$CAMERA_DEVICE"
  printf 'CAMERA_HOST=%s\n' "$CAMERA_HOST"
  printf 'CAMERA_PORT=%s\n' "$CAMERA_PORT"
  printf 'CAMERA_RESOLUTION=%s\n' "$CAMERA_RESOLUTION"
  printf 'CAMERA_DYNAMIC_FRAMERATE=%s\n' "$CAMERA_DYNAMIC_FRAMERATE"
} > "$ENV_FILE"

run install -d -m 0755 "$CONFIG_DIR"
run install -o root -g root -m 0644 "$ENV_FILE" "$CONFIG_DIR/camera.env"
run install -o root -g root -m 0644 "$REPOSITORY_ROOT/deploy/systemd/$SERVICE_NAME" "/etc/systemd/system/$SERVICE_NAME"
run systemctl daemon-reload
run systemctl enable "$SERVICE_NAME"
if "$NO_START"; then
  printf 'Camera stream installed without starting it.\n'
else
  run systemctl restart "$SERVICE_NAME"
  printf 'Camera stream: http://%s:%s/stream\n' "$CAMERA_HOST" "$CAMERA_PORT"
fi
