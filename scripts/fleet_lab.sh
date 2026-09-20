#!/usr/bin/env bash
# Start isolated Agent simulators for control-plane development.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PACKAGE_DIR="${REPO_DIR}/robot_agent/amr_web_bridge"
PROFILE_DIR="${PACKAGE_DIR}/config/fleet_lab"
STATE_DIR="${FLEET_LAB_STATE_DIR:-${XDG_STATE_HOME:-${HOME}/.local/state}/indoor-delivery-fleet-lab}"
SERVER_URL="${ROBOT_CONTROL_URL:-http://localhost:8000}"
WEB_REPO="${INDOOR_DELIVERY_WEB_REPO:-${HOME}/indoor-delivery-robot}"
SESSION_NAME="indoor-delivery-fleet-lab"
agents=(sim01 sim02 robot-test01)

read_enrollment_token() {
  [[ -n "${ROBOT_ENROLLMENT_TOKEN:-}" ]] && { printf '%s' "${ROBOT_ENROLLMENT_TOKEN}"; return; }
  local env_file="${WEB_REPO}/backend/.env"
  [[ -f "${env_file}" ]] && sed -n 's/^ROBOT_ENROLLMENT_TOKEN=//p' "${env_file}" | tail -n 1
}
is_running() { [[ "$(tmux list-panes -t "${SESSION_NAME}:$1" -F '#{pane_dead}' 2>/dev/null || true)" == "0" ]]; }
ensure_session() {
  tmux has-session -t "${SESSION_NAME}" 2>/dev/null || tmux new-session -d -s "${SESSION_NAME}" -n control 'sleep infinity'
  tmux set-environment -t "${SESSION_NAME}" ROBOT_ENROLLMENT_TOKEN "$1"
  tmux set-environment -t "${SESSION_NAME}" ROS_DISTRO "${ROS_DISTRO:-jazzy}"
  tmux set-environment -t "${SESSION_NAME}" PYTHONPATH "${PACKAGE_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
}
start_agent() {
  local agent="$1" token command
  is_running "$agent" && { printf '%-14s already running\n' "$agent"; return; }
  token="$(read_enrollment_token)"; [[ -n "$token" ]] || { echo "ROBOT_ENROLLMENT_TOKEN is not set and ${WEB_REPO}/backend/.env has no token" >&2; exit 1; }
  mkdir -p "${STATE_DIR}/credentials" "${STATE_DIR}/logs"; ensure_session "$token"
  printf -v command 'python3 -m amr_web_bridge.fleet_agent_simulator --profile %q --server-url %q --credential-file %q >>%q 2>&1' "${PROFILE_DIR}/${agent}.yaml" "${SERVER_URL}" "${STATE_DIR}/credentials/${agent}.json" "${STATE_DIR}/logs/${agent}.log"
  tmux new-window -d -t "${SESSION_NAME}" -n "$agent" "$command"; tmux set-option -t "${SESSION_NAME}:${agent}" remain-on-exit on
  printf '%-14s started\n' "$agent"
}
stop_agent() { tmux kill-window -t "${SESSION_NAME}:$1" 2>/dev/null || true; printf '%-14s stopped\n' "$1"; }
status_agent() { if is_running "$1"; then printf '%-14s running  credential=%s\n' "$1" "$([[ -f "${STATE_DIR}/credentials/$1.json" ]] && echo issued || echo pending)"; else printf '%-14s stopped  credential=%s\n' "$1" "$([[ -f "${STATE_DIR}/credentials/$1.json" ]] && echo issued || echo pending)"; fi; }
selected_agents() {
  [[ "${1:-all}" == all ]] && { printf '%s\n' "${agents[@]}"; return; }
  for agent in "${agents[@]}"; do [[ "$agent" == "$1" ]] && { printf '%s\n' "$agent"; return; }; done
  echo "Unknown agent '$1'. Expected: ${agents[*]} or all" >&2; exit 2
}
action="${1:-status}"; target="${2:-all}"; mapfile -t selected < <(selected_agents "$target")
case "$action" in
  start) for agent in "${selected[@]}"; do start_agent "$agent"; done; echo "Approve pending requests in http://localhost:3000/robots" ;;
  stop) for agent in "${selected[@]}"; do stop_agent "$agent"; done ;;
  restart) for agent in "${selected[@]}"; do stop_agent "$agent"; start_agent "$agent"; done ;;
  status) for agent in "${selected[@]}"; do status_agent "$agent"; done ;;
  logs) [[ "$target" != all ]] || { echo "logs requires one agent: ${agents[*]}" >&2; exit 2; }; tail -n 80 -f "${STATE_DIR}/logs/${target}.log" ;;
  reset-credentials) for agent in "${selected[@]}"; do stop_agent "$agent"; rm -f "${STATE_DIR}/credentials/${agent}.json"; printf '%-14s credential removed\n' "$agent"; done ;;
  *) echo "Usage: $0 {start|stop|restart|status|logs|reset-credentials} [sim01|sim02|robot-test01|all]" >&2; exit 2 ;;
esac
