from __future__ import annotations

import json
import re
import subprocess
from typing import Any, Callable


NAVIGATION_NODES = (
    '/planner_server',
    '/controller_server',
    '/bt_navigator',
)

# A fresh Fast DDS command-line client can take several seconds to discover a
# service after the navigation stack starts. Six seconds was short enough to
# report a failed reset even though the serial bridge completed it moments
# later. Keep this bounded, but allow the one STOP/CLEAR request to complete.
MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS = 15.0

# Every ``ros2 service call`` below starts a new CLI process. On the ODROID a
# fresh Fast DDS participant can need more than the old eight-second default to
# discover Nav2's costmap services, even while the services are healthy.
COSTMAP_CLEAR_TIMEOUT_SECONDS = 15.0


def classify_plan_failure(output: str) -> str:
    normalized = output.lower()
    if 'timed out' in normalized or 'timeout' in normalized:
        return 'Planner timeout'
    if any(value in normalized for value in (
        'start occupied', 'start is occupied', 'start pose is invalid',
        'start_outside_map', 'start occupied by obstacle',
        'error_code=203', 'error_code: 203',
        'error_code=205', 'error_code: 205',
    )):
        return 'Start blocked'
    if any(value in normalized for value in (
        'goal occupied', 'goal is occupied', 'goal pose is invalid',
        'goal_outside_map', 'goal occupied by obstacle',
        'error_code=204', 'error_code: 204',
        'error_code=206', 'error_code: 206',
    )):
        return 'Goal blocked'
    return 'No connected path'


class NavigationRecoveryRunner:
    """Runs bounded Nav2 recovery checks without accepting arbitrary shell input."""

    def __init__(
        self,
        *,
        run: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
        control_helper: str = '/usr/local/sbin/indoor-delivery-robot-control',
    ) -> None:
        self._run = run
        self.control_helper = control_helper

    def _command(self, arguments: list[str], timeout: float = 8.0) -> tuple[bool, str]:
        try:
            result = self._run(
                arguments,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            return False, str(error)
        output = '\n'.join(value for value in (result.stdout, result.stderr) if value).strip()
        return result.returncode == 0, output

    def lifecycle_active(self, node: str) -> tuple[bool, str]:
        ok, output = self._command(['ros2', 'lifecycle', 'get', node], timeout=4.0)
        # "inactive" must never satisfy the active-state gate.
        active = re.search(r'^\s*active\s*\[3\]\s*$', output, re.MULTILINE)
        return ok and active is not None, output

    def nav2_lifecycle_healthy(self) -> tuple[bool, str]:
        details = []
        for node in NAVIGATION_NODES:
            active, output = self.lifecycle_active(node)
            details.append(f'{node}: {output or "unavailable"}')
            if not active:
                return False, '; '.join(details)
        return True, '; '.join(details)

    def lifecycle_manager_available(self) -> tuple[bool, str]:
        ok, output = self._command([
            'ros2', 'service', 'type',
            '/lifecycle_manager_navigation/manage_nodes',
        ], timeout=4.0)
        available = ok and 'nav2_msgs/srv/ManageLifecycleNodes' in output
        return available, output

    def clear_costmaps(self) -> tuple[bool, str]:
        for service in (
            '/local_costmap/clear_entirely_local_costmap',
            '/global_costmap/clear_entirely_global_costmap',
        ):
            ok, output = self._command([
                'ros2', 'service', 'call', service,
                'nav2_msgs/srv/ClearEntireCostmap', '{}',
            ], timeout=COSTMAP_CLEAR_TIMEOUT_SECONDS)
            if not ok:
                return False, f'{service}: {output or "service call failed"}'
        return True, 'Local and global costmaps cleared'

    def compute_path(self, target: dict[str, Any]) -> tuple[bool, str]:
        goal = {
            'goal': {
                'header': {'frame_id': str(target['frame_id'])},
                'pose': {
                    'position': {
                        'x': float(target['x']),
                        'y': float(target['y']),
                        'z': 0.0,
                    },
                    'orientation': {
                        'z': __import__('math').sin(float(target['yaw']) / 2.0),
                        'w': __import__('math').cos(float(target['yaw']) / 2.0),
                    },
                },
            },
            'planner_id': 'GridBased',
            'use_start': False,
        }
        ok, output = self._command([
            'ros2', 'action', 'send_goal',
            '/compute_path_to_pose',
            'nav2_msgs/action/ComputePathToPose',
            json.dumps(goal, separators=(',', ':')),
        ], timeout=15.0)
        succeeded = ok and (
            'status: succeeded' in output.lower()
            or 'error_code=0' in output.lower()
            or 'error_code: 0' in output.lower()
        )
        return succeeded, output

    def clear_motor_fault(self) -> tuple[bool, str]:
        ok, output = self._command([
            'ros2', 'service', 'call',
            '/clear_motor_fault',
            'std_srvs/srv/Trigger', '{}',
        ], timeout=MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS)
        # ros2 CLI output differs between releases: Jazzy prints a Python
        # dataclass representation (``success=True``), while other versions
        # may print YAML (``success: true``). Both are the same successful
        # Trigger response.
        normalized = output.lower().replace(' ', '')
        succeeded = ok and (
            'success=true' in normalized
            or 'success:true' in normalized
        )
        return succeeded, output

    def finish_motor_recovery(self, success: bool) -> tuple[bool, str]:
        ok, output = self._command([
            'ros2', 'service', 'call',
            '/finish_motor_recovery',
            'std_srvs/srv/SetBool',
            json.dumps({'data': bool(success)}, separators=(',', ':')),
        ], timeout=MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS)
        normalized = output.lower().replace(' ', '')
        succeeded = ok and (
            'success=true' in normalized
            or 'success:true' in normalized
        )
        return succeeded, output

    def back_up(self, distance: float = 0.10) -> tuple[bool, str]:
        goal = {
            'target': {'x': abs(float(distance)), 'y': 0.0, 'z': 0.0},
            'speed': 0.05,
            'time_allowance': {'sec': 4, 'nanosec': 0},
        }
        return self._behavior_action(
            '/backup', 'nav2_msgs/action/BackUp', goal, timeout=15.0)

    def guarded_reverse(self) -> tuple[bool, str]:
        """Request the fixed scan-guarded escape outside Nav2's stale costmap."""
        ok, output = self._command([
            'ros2', 'service', 'call',
            '/guarded_reverse_escape',
            'std_srvs/srv/Trigger', '{}',
        ], timeout=MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS)
        normalized = output.lower().replace(' ', '')
        succeeded = ok and (
            'success=true' in normalized
            or 'success:true' in normalized
        )
        return succeeded, output

    def guarded_arc(self, yaw: float) -> tuple[bool, str]:
        """Request a fixed scan/odometry-guarded rolling reverse arc."""
        side = 'left' if float(yaw) > 0.0 else 'right'
        ok, output = self._command([
            'ros2', 'service', 'call',
            f'/guarded_arc_escape_{side}',
            'std_srvs/srv/Trigger', '{}',
        ], timeout=MOTOR_FAULT_CLEAR_TIMEOUT_SECONDS)
        normalized = output.lower().replace(' ', '')
        succeeded = ok and (
            'success=true' in normalized
            or 'success:true' in normalized
        )
        return succeeded, output

    def spin(self, yaw: float) -> tuple[bool, str]:
        goal = {
            'target_yaw': float(yaw),
            'time_allowance': {'sec': 5, 'nanosec': 0},
        }
        return self._behavior_action(
            '/spin', 'nav2_msgs/action/Spin', goal, timeout=15.0)

    def _behavior_action(
        self,
        name: str,
        action_type: str,
        goal: dict[str, Any],
        *,
        timeout: float,
    ) -> tuple[bool, str]:
        ok, output = self._command([
            'ros2', 'action', 'send_goal', name, action_type,
            json.dumps(goal, separators=(',', ':')),
        ], timeout=timeout)
        normalized = output.lower().replace(' ', '')
        succeeded = ok and (
            'status:succeeded' in normalized
            or 'error_code=0' in normalized
            or 'error_code:0' in normalized
        )
        return succeeded, output

    def system_action(self, action: str) -> tuple[bool, str]:
        if action not in {
            'start-navigation', 'restart-navigation',
            'stop-navigation', 'poweroff',
        }:
            return False, 'Unsupported system action'
        return self._command(['sudo', self.control_helper, action], timeout=12.0)

    def activate_map(self, map_yaml: str) -> tuple[bool, str]:
        """Persist one validated map and start Nav2 through the root helper."""
        return self._command(
            ['sudo', self.control_helper, 'activate-map', map_yaml],
            timeout=18.0,
        )
