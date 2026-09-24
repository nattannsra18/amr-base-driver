from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
from threading import Lock
import time
from typing import Any, Callable

from .map_catalog import available_map_yaml, MAP_ID_PATTERN, update_map_metadata


class MappingRuntime:
    """Own the trusted ROS processes used by one web mapping session."""

    def __init__(
        self,
        maps_directory: str,
        *,
        slam_params_file: str | None = None,
        run: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
        popen: Callable[..., subprocess.Popen[Any]] = subprocess.Popen,
        sleep: Callable[[float], None] = time.sleep,
        lifecycle_state: Callable[[str], bool] | None = None,
        lifecycle_command: Callable[[str, int], None] | None = None,
    ) -> None:
        self.maps_directory = maps_directory
        self.slam_params_file = slam_params_file
        self._run = run
        self._popen = popen
        self._sleep = sleep
        self._lifecycle_state = lifecycle_state
        self._lifecycle_command = lifecycle_command
        self._lock = Lock()
        self._process: subprocess.Popen[Any] | None = None
        self._session_id: str | None = None
        self._phase = 'IDLE'
        self._detail: str | None = None
        self._started_at: str | None = None
        self._saved_map_id: str | None = None
        self._start_map_revision: int | None = None
        self._navigation_was_active = False
        self._localization_was_active = False

    def snapshot(self, map_revision: int) -> dict[str, Any]:
        with self._lock:
            return {
                'type': 'mapping_status',
                'robot_id': '',
                'session_id': self._session_id,
                'phase': self._phase,
                'detail': self._detail,
                'started_at': self._started_at,
                'saved_map_id': self._saved_map_id,
                'map_revision': map_revision,
            }

    def _set(self, phase: str, detail: str | None = None) -> None:
        with self._lock:
            self._phase = phase
            self._detail = detail

    def _ros(self, arguments: list[str], timeout: float = 8.0) -> str:
        try:
            result = self._run(
                ['ros2', *arguments],
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as error:
            command = ' '.join(['ros2', *arguments])
            raise RuntimeError(
                f'ROS command timed out after {timeout:.0f}s: {command}'
            ) from error
        output = '\n'.join(part.strip() for part in (result.stdout, result.stderr) if part.strip())
        if result.returncode != 0:
            raise RuntimeError(
                output
                or f'ROS command failed with exit code {result.returncode}'
            )
        return output

    def _lifecycle(
        self,
        manager: str,
        command: int,
        *,
        timeout: float = 8.0,
    ) -> None:
        if self._lifecycle_command is not None:
            self._lifecycle_command(manager, command)
            return
        output = self._ros([
            'service', 'call', f'/{manager}/manage_nodes',
            'nav2_msgs/srv/ManageLifecycleNodes', f'{{command: {command}}}',
        ], timeout=timeout)
        if 'success=True' not in output and 'success: true' not in output.lower():
            raise RuntimeError(f'{manager} did not confirm the lifecycle transition')

    def _lifecycle_node_active(self, node: str) -> bool:
        if self._lifecycle_state is not None:
            return self._lifecycle_state(node)
        output = self._ros(
            ['lifecycle', 'get', f'/{node}'],
            # A freshly restarted ROS graph on the ODROID can take more than
            # four seconds to discover lifecycle services. The two checks run
            # concurrently, so this is one bounded discovery window.
            timeout=12.0,
        )
        return output.strip().lower().startswith('active ')

    def _active_lifecycle_nodes(self) -> tuple[bool, bool]:
        """Query Nav2 state concurrently so DDS discovery gets one short budget."""
        with ThreadPoolExecutor(max_workers=2) as executor:
            navigation = executor.submit(
                self._lifecycle_node_active, 'planner_server',
            )
            localization = executor.submit(
                self._lifecycle_node_active, 'amcl',
            )
            return navigation.result(), localization.result()

    def _ensure_lifecycle_started(
        self,
        manager: str,
        node: str,
        *,
        attempts: int = 3,
    ) -> None:
        """Start a lifecycle manager, tolerating a lost service response."""
        last_error: Exception | None = None
        for attempt in range(attempts):
            try:
                self._lifecycle(manager, 2)
                return
            except Exception as error:
                last_error = error
                try:
                    if self._lifecycle_node_active(node):
                        return
                except Exception as state_error:
                    last_error = state_error
                if attempt + 1 < attempts:
                    self._sleep(1.0)
        raise RuntimeError(
            f'{manager} failed to restore {node} after {attempts} attempts: '
            f'{last_error}'
        ) from last_error

    def start(
        self,
        session_id: str,
        initial_map_revision: int | None = None,
    ) -> None:
        with self._lock:
            if self._phase not in {'IDLE', 'FAILED'}:
                raise ValueError('A mapping session is already active')
            self._phase = 'STARTING'
            self._session_id = session_id
            self._started_at = datetime.now(timezone.utc).isoformat()
            self._saved_map_id = None
            self._start_map_revision = initial_map_revision
            self._detail = 'Checking Nav2 lifecycle readiness'
        navigation_paused = False
        localization_paused = False
        try:
            navigation_active, localization_active = (
                self._active_lifecycle_nodes()
            )
            with self._lock:
                self._navigation_was_active = navigation_active
                self._localization_was_active = localization_active
            self._set('STARTING', 'Pausing active Nav2 lifecycle managers')
            # Pause sequentially so a failure in the second transition cannot
            # hide the first successful pause from the rollback path.
            if navigation_active:
                self._lifecycle('lifecycle_manager_navigation', 1)
                navigation_paused = True
            if localization_active:
                self._lifecycle('lifecycle_manager_localization', 1)
                localization_paused = True
            self._set('STARTING', 'Starting SLAM Toolbox')
            launch_arguments = [
                # Synchronous scan processing avoids an optimization backlog
                # while the robot is driven interactively from the web UI.
                # That backlog can make the live map appear warped even when
                # the final saved graph is optimized correctly.
                'ros2', 'launch', 'slam_toolbox', 'online_sync_launch.py',
                'use_sim_time:=false', 'autostart:=true',
            ]
            if self.slam_params_file:
                launch_arguments.append(
                    f'slam_params_file:={self.slam_params_file}'
                )
            process = self._popen(
                launch_arguments,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self._sleep(2.0)
            if process.poll() is not None:
                raise RuntimeError('SLAM Toolbox exited before mapping became ready')
            with self._lock:
                self._process = process
                self._phase = 'MAPPING'
                self._detail = 'SLAM Toolbox is building a live occupancy map'
        except Exception as error:
            if localization_paused:
                self._safe_lifecycle('lifecycle_manager_localization', 2)
            if navigation_paused:
                self._safe_lifecycle('lifecycle_manager_navigation', 2)
            with self._lock:
                self._localization_was_active = False
                self._navigation_was_active = False
            self._set(
                'FAILED',
                f'Unable to enter ROS mapping mode: {error}',
            )
            raise

    def stop_capture(self) -> None:
        with self._lock:
            if self._phase != 'MAPPING':
                raise ValueError('Mapping is not active')
        # Motion is stopped by WebBridgeNode before this transition and the
        # dead-man timer continues to enforce zero velocity. Keeping SLAM
        # active preserves its in-memory map for Save without relying on the
        # pause service, whose CLI client can block intermittently under DDS.
        self._set('REVIEW', 'Robot stopped; captured map is ready for review')

    def save(
        self,
        map_id: str,
        metadata: dict[str, Any],
        occupancy_map: dict[str, Any] | None = None,
        activate_saved_map: Callable[[Path], None] | None = None,
    ) -> None:
        with self._lock:
            if self._phase != 'REVIEW':
                raise ValueError('Stop map capture before saving')
            self._phase = 'SAVING'
            self._detail = 'Saving map files on the robot'
        if MAP_ID_PATTERN.fullmatch(map_id) is None:
            self._set('REVIEW', 'Map ID is invalid')
            raise ValueError('Map ID is invalid')
        directory_value = self.maps_directory.strip()
        if not directory_value:
            self._set('REVIEW', 'Map directory is not configured')
            raise ValueError('Map directory is not configured')
        directory = Path(directory_value).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        target = (directory / map_id).resolve()
        if not target.is_relative_to(directory):
            self._set('REVIEW', 'Map path is outside the configured directory')
            raise ValueError('Map path is outside the configured directory')
        if any((directory / f'{map_id}{suffix}').exists() for suffix in ('.yaml', '.pgm', '.png')):
            self._set('REVIEW', 'Map ID already exists')
            raise ValueError('Map ID already exists')
        try:
            if occupancy_map is None:
                self._ros([
                    'service', 'call', '/slam_toolbox/save_map',
                    'slam_toolbox/srv/SaveMap',
                    json.dumps({'name': {'data': str(target)}}),
                ], timeout=30.0)
            else:
                self._write_occupancy_map(target, occupancy_map)
            yaml_path = available_map_yaml(self.maps_directory, map_id)
            if yaml_path is None:
                raise RuntimeError('Saved map files failed validation')
            update_map_metadata(self.maps_directory, map_id, metadata)
            with self._lock:
                self._saved_map_id = map_id
            if activate_saved_map is None:
                self._restore()
                detail = 'Map saved and Nav2 localization restored'
            else:
                self._stop_slam_process()
                activate_saved_map(yaml_path)
                with self._lock:
                    self._localization_was_active = False
                    self._navigation_was_active = False
                detail = 'Map saved and activated; set the Initial Pose to localize'
            self._set('IDLE', detail)
        except Exception:
            with self._lock:
                saved = self._saved_map_id is not None
            self._set(
                'FAILED' if saved else 'REVIEW',
                (
                    'Map was saved, but Nav2 restoration needs attention'
                    if saved else
                    'Map save failed; the captured map remains available for retry'
                ),
            )
            raise

    def _write_occupancy_map(
        self,
        target: Path,
        occupancy_map: dict[str, Any],
    ) -> None:
        """
        Persist the exact live SLAM frame already received by the agent.

        Calling ``/slam_toolbox/save_map`` through a newly discovered ROS CLI
        participant can take tens of seconds on the ODROID and, when an old
        transient-local ``/map`` sample is still present, has saved that stale
        map.  The agent already owns the current OccupancyGrid, so write that
        immutable snapshot directly and atomically.
        """
        width = int(occupancy_map.get('width', 0))
        height = int(occupancy_map.get('height', 0))
        resolution = float(occupancy_map.get('resolution', 0.0))
        data = occupancy_map.get('data')
        revision = occupancy_map.get('revision')
        if (
            width <= 0
            or height <= 0
            or resolution <= 0.0
            or not isinstance(data, list)
            or len(data) != width * height
        ):
            raise RuntimeError('The live SLAM map is incomplete')
        if (
            self._start_map_revision is not None
            and isinstance(revision, int)
            and revision <= self._start_map_revision
        ):
            raise RuntimeError('Waiting for the first map from this SLAM session')

        pgm_path = target.with_suffix('.pgm')
        yaml_path = target.with_suffix('.yaml')
        pgm_temp = pgm_path.with_suffix('.pgm.tmp')
        yaml_temp = yaml_path.with_suffix('.yaml.tmp')
        pixels = bytearray()
        for row in range(height - 1, -1, -1):
            offset = row * width
            for value in data[offset:offset + width]:
                occupancy = int(value)
                if occupancy < 0:
                    pixels.append(205)
                elif occupancy >= 65:
                    pixels.append(0)
                elif occupancy <= 25:
                    pixels.append(254)
                else:
                    pixels.append(205)
        pgm_temp.write_bytes(
            f'P5\n{width} {height}\n255\n'.encode('ascii') + pixels
        )
        origin = [
            float(occupancy_map.get('origin_x', 0.0)),
            float(occupancy_map.get('origin_y', 0.0)),
            float(occupancy_map.get('origin_yaw', 0.0)),
        ]
        yaml_temp.write_text(
            '\n'.join((
                f'image: {pgm_path.name}',
                'mode: trinary',
                f'resolution: {resolution}',
                f'origin: [{origin[0]}, {origin[1]}, {origin[2]}]',
                'negate: 0',
                'occupied_thresh: 0.65',
                'free_thresh: 0.25',
                '',
            )),
            encoding='utf-8',
        )
        os.replace(pgm_temp, pgm_path)
        os.replace(yaml_temp, yaml_path)

    def discard(self) -> None:
        with self._lock:
            if self._phase not in {'MAPPING', 'REVIEW', 'FAILED'}:
                raise ValueError('No mapping session can be discarded')
            self._phase = 'RESTORING'
            self._detail = 'Restoring Nav2 localization'
        try:
            self._restore()
        except Exception as error:
            self._set('FAILED', f'Unable to restore Nav2 localization: {error}')
            raise
        with self._lock:
            self._session_id = None
            self._started_at = None
            self._saved_map_id = None
            self._phase = 'IDLE'
            self._detail = 'Captured map discarded and Nav2 localization restored'

    def _restore(self) -> None:
        self._stop_slam_process()
        with self._lock:
            localization_was_active = self._localization_was_active
            navigation_was_active = self._navigation_was_active
        self._resume_lifecycles(
            localization_was_active,
            navigation_was_active,
        )

    def _stop_slam_process(self) -> None:
        with self._lock:
            process = self._process
            self._process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=8.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.0)

    def _resume_lifecycles(
        self,
        localization_was_active: bool,
        navigation_was_active: bool,
    ) -> None:
        if localization_was_active:
            self._ensure_lifecycle_started(
                'lifecycle_manager_localization', 'amcl',
            )
            with self._lock:
                self._localization_was_active = False
        if navigation_was_active:
            self._ensure_lifecycle_started(
                'lifecycle_manager_navigation', 'planner_server',
            )
            with self._lock:
                self._navigation_was_active = False

    def _safe_lifecycle(self, manager: str, command: int) -> None:
        try:
            self._lifecycle(manager, command)
        except Exception:
            pass

    def shutdown(self) -> None:
        with self._lock:
            process = self._process
        if process is not None:
            try:
                self._restore()
            except Exception:
                pass
