from __future__ import annotations

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
    ) -> None:
        self.maps_directory = maps_directory
        self.slam_params_file = slam_params_file
        self._run = run
        self._popen = popen
        self._sleep = sleep
        self._lock = Lock()
        self._process: subprocess.Popen[Any] | None = None
        self._session_id: str | None = None
        self._phase = 'IDLE'
        self._detail: str | None = None
        self._started_at: str | None = None
        self._saved_map_id: str | None = None

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

    def _ros(self, arguments: list[str], timeout: float = 20.0) -> str:
        result = self._run(
            ['ros2', *arguments],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = '\n'.join(part.strip() for part in (result.stdout, result.stderr) if part.strip())
        if result.returncode != 0:
            raise RuntimeError(
                output
                or f'ROS command failed with exit code {result.returncode}'
            )
        return output

    def _lifecycle(self, manager: str, command: int) -> None:
        output = self._ros([
            'service', 'call', f'/{manager}/manage_nodes',
            'nav2_msgs/srv/ManageLifecycleNodes', f'{{command: {command}}}',
        ])
        if 'success=True' not in output and 'success: true' not in output.lower():
            raise RuntimeError(f'{manager} did not confirm the lifecycle transition')

    def start(self, session_id: str) -> None:
        with self._lock:
            if self._phase not in {'IDLE', 'FAILED'}:
                raise ValueError('A mapping session is already active')
            self._phase = 'STARTING'
            self._session_id = session_id
            self._started_at = datetime.now(timezone.utc).isoformat()
            self._saved_map_id = None
            self._detail = 'Pausing Nav2 localization and navigation'
        navigation_paused = False
        localization_paused = False
        try:
            self._lifecycle('lifecycle_manager_navigation', 1)
            navigation_paused = True
            self._lifecycle('lifecycle_manager_localization', 1)
            localization_paused = True
            launch_arguments = [
                # Synchronous scan processing avoids an optimization backlog
                # while the robot is driven interactively from the web UI.
                # That backlog can make the live map appear warped even when
                # the final saved graph is optimized correctly.
                'ros2', 'launch', 'slam_toolbox', 'online_sync_launch.py',
                'use_sim_time:=true', 'autostart:=true',
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
        except Exception:
            if localization_paused:
                self._safe_lifecycle('lifecycle_manager_localization', 2)
            if navigation_paused:
                self._safe_lifecycle('lifecycle_manager_navigation', 2)
            self._set('FAILED', 'Unable to enter ROS mapping mode')
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

    def save(self, map_id: str, metadata: dict[str, Any]) -> None:
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
            self._ros([
                'service', 'call', '/slam_toolbox/save_map',
                'slam_toolbox/srv/SaveMap',
                json.dumps({'name': {'data': str(target)}}),
            ], timeout=30.0)
            yaml_path = available_map_yaml(self.maps_directory, map_id)
            if yaml_path is None:
                raise RuntimeError('Saved map files failed validation')
            update_map_metadata(self.maps_directory, map_id, metadata)
            with self._lock:
                self._saved_map_id = map_id
            self._restore()
            self._set('IDLE', 'Map saved and Nav2 localization restored')
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

    def discard(self) -> None:
        with self._lock:
            if self._phase not in {'MAPPING', 'REVIEW', 'FAILED'}:
                raise ValueError('No mapping session can be discarded')
            self._phase = 'RESTORING'
            self._detail = 'Restoring Nav2 localization'
        self._restore()
        with self._lock:
            self._session_id = None
            self._started_at = None
            self._saved_map_id = None
            self._phase = 'IDLE'
            self._detail = 'Captured map discarded and Nav2 localization restored'

    def _restore(self) -> None:
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
        self._lifecycle('lifecycle_manager_localization', 2)
        self._lifecycle('lifecycle_manager_navigation', 2)

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
