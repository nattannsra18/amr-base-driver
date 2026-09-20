"""Headless Robot Agent used to exercise the control plane without an SBC."""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
from typing import Any
from uuid import uuid4

import websockets
import yaml

from .agent_identity import AgentCredentialStore, EnrollmentClient, EnrollmentError


def utc_timestamp() -> str:
    """Return an Agent Protocol compatible UTC timestamp."""
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class FleetAgentProfile:
    """Identity and behavior of one local test agent."""

    serial_number: str
    display_name: str
    profile_version: str
    ros_namespace: str
    active_map_id: str
    capabilities: list[str]
    battery_percent: int
    initial_x: float
    initial_y: float
    initial_yaw: float
    navigation_delay_seconds: float

    @classmethod
    def load(cls, path: Path) -> 'FleetAgentProfile':
        payload = yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(payload, dict):
            raise ValueError('Fleet Agent profile must be a YAML object')
        identity = payload.get('identity', {})
        ros = payload.get('ros', {})
        simulation = payload.get('simulation', {})
        capabilities = sorted({
            str(item).strip().lower()
            for item in payload.get('capabilities', [])
            if str(item).strip()
        })
        required = {
            'serial_number': identity.get('serial_number'),
            'display_name': identity.get('display_name'),
            'profile_version': identity.get('profile_version'),
            'ros_namespace': ros.get('namespace'),
            'active_map_id': ros.get('active_map_id'),
        }
        missing = [name for name, value in required.items() if not value]
        if missing or not capabilities:
            fields = missing + ([] if capabilities else ['capabilities'])
            raise ValueError(f"Missing Fleet Agent profile fields: {', '.join(fields)}")
        namespace = str(required['ros_namespace'])
        if not namespace.startswith('/') or namespace == '/':
            raise ValueError('ros.namespace must be a non-root absolute namespace')
        battery = int(simulation.get('battery_percent', 100))
        if not 0 <= battery <= 100:
            raise ValueError('simulation.battery_percent must be between 0 and 100')
        return cls(
            serial_number=str(required['serial_number']),
            display_name=str(required['display_name']),
            profile_version=str(required['profile_version']),
            ros_namespace=namespace.rstrip('/'),
            active_map_id=str(required['active_map_id']),
            capabilities=capabilities,
            battery_percent=battery,
            initial_x=float(simulation.get('x', 0.0)),
            initial_y=float(simulation.get('y', 0.0)),
            initial_yaw=float(simulation.get('yaw', 0.0)),
            navigation_delay_seconds=max(
                0.1, float(simulation.get('navigation_delay_seconds', 2.0))
            ),
        )


class FleetAgentSimulator:
    """Enroll, connect and behave like one minimal navigation-capable agent."""

    def __init__(
        self,
        profile: FleetAgentProfile,
        server_url: str,
        credential_file: Path,
        bootstrap_token: str,
    ) -> None:
        self.profile = profile
        self.server_url = server_url.rstrip('/')
        self.credential_store = AgentCredentialStore(credential_file)
        self.bootstrap_token = bootstrap_token
        self.stop_requested = asyncio.Event()
        self.robot_id = ''
        self.credential = ''
        self.boot_id = str(uuid4())
        self.x = profile.initial_x
        self.y = profile.initial_y
        self.yaw = profile.initial_yaw

    async def run(self) -> None:
        while not self.stop_requested.is_set():
            try:
                await self._ensure_identity()
                await self._connect_once()
            except (EnrollmentError, OSError, websockets.WebSocketException) as error:
                print(f'[{self.profile.serial_number}] disconnected: {error}', flush=True)
            if not self.stop_requested.is_set():
                await asyncio.sleep(2.0)

    def stop(self) -> None:
        self.stop_requested.set()

    async def _ensure_identity(self) -> None:
        stored = self.credential_store.load()
        if stored is not None:
            self.robot_id = stored.robot_id
            self.credential = stored.credential
            return
        if not self.bootstrap_token:
            raise EnrollmentError(0, 'ROBOT_ENROLLMENT_TOKEN is required before pairing')
        client = EnrollmentClient(self.server_url, self.bootstrap_token)
        fingerprint = f'fleet-lab:{self.profile.serial_number}:{self.profile.ros_namespace}'
        enrollment_payload = {
            'serial_number': self.profile.serial_number,
            'hardware_fingerprint': fingerprint,
            'display_name': self.profile.display_name,
            'agent_version': 'fleet-lab-1.0',
            'ros_distro': os.getenv('ROS_DISTRO', 'jazzy'),
            'profile_version': self.profile.profile_version,
            'capabilities': self.profile.capabilities,
        }
        while not self.stop_requested.is_set():
            try:
                enrollment = await asyncio.to_thread(
                    client.create,
                    enrollment_payload,
                )
                break
            except EnrollmentError as error:
                if error.status != 429:
                    raise
                delay = max(1, error.retry_after_seconds or 3)
                print(
                    f'[{self.profile.serial_number}] enrollment rate limited; '
                    f'retrying in {delay} s',
                    flush=True,
                )
                await asyncio.sleep(delay)
        else:
            return
        print(
            f'[{self.profile.serial_number}] PAIRING CODE {enrollment.pairing_code} '
            f'(namespace {self.profile.ros_namespace})',
            flush=True,
        )
        while not self.stop_requested.is_set():
            try:
                credential = await asyncio.to_thread(
                    client.claim,
                    enrollment.enrollment_id,
                    enrollment.pairing_code,
                    fingerprint,
                )
            except EnrollmentError as error:
                if error.status == 409:
                    await asyncio.sleep(enrollment.poll_after_seconds)
                    continue
                if error.status == 429:
                    delay = max(
                        enrollment.poll_after_seconds,
                        error.retry_after_seconds or 0,
                    )
                    print(
                        f'[{self.profile.serial_number}] claim rate limited; '
                        f'keeping the current code and retrying in {delay} s',
                        flush=True,
                    )
                    await asyncio.sleep(delay)
                    continue
                raise
            await asyncio.to_thread(self.credential_store.save, credential)
            self.robot_id = credential.robot_id
            self.credential = credential.credential
            print(
                f'[{self.profile.serial_number}] paired as {self.robot_id}; '
                f'credential saved to {self.credential_store.path}',
                flush=True,
            )
            return

    async def _connect_once(self) -> None:
        websocket_url = self.server_url
        if websocket_url.startswith('http://'):
            websocket_url = f'ws://{websocket_url[7:]}'
        elif websocket_url.startswith('https://'):
            websocket_url = f'wss://{websocket_url[8:]}'
        uri = f'{websocket_url}/ws/robots/{self.robot_id}'
        async with websockets.connect(
            uri,
            extra_headers={'Authorization': f'Bearer {self.credential}'},
            open_timeout=5,
            ping_interval=20,
            ping_timeout=20,
        ) as websocket:
            await self._send(websocket, {
                'type': 'agent_hello',
                'protocol_version': '1.0',
                'robot_id': self.robot_id,
                'boot_id': self.boot_id,
                'agent_version': 'fleet-lab-1.0',
                'ros_distro': os.getenv('ROS_DISTRO', 'jazzy'),
                'profile_version': self.profile.profile_version,
                'capabilities': self.profile.capabilities,
            })
            await self._send_readiness(websocket)
            print(
                f'[{self.profile.serial_number}] connected as {self.robot_id} '
                f'on {self.profile.active_map_id}',
                flush=True,
            )
            heartbeat = asyncio.create_task(self._heartbeat_loop(websocket))
            telemetry = asyncio.create_task(self._telemetry_loop(websocket))
            try:
                async for raw_message in websocket:
                    message = json.loads(raw_message)
                    if isinstance(message, dict):
                        await self._handle_message(websocket, message)
            finally:
                heartbeat.cancel()
                telemetry.cancel()
                await asyncio.gather(heartbeat, telemetry, return_exceptions=True)

    async def _heartbeat_loop(self, websocket: Any) -> None:
        while True:
            await asyncio.sleep(2.0)
            await self._send(websocket, {'type': 'heartbeat', 'timestamp': utc_timestamp()})
            await self._send_readiness(websocket)

    async def _telemetry_loop(self, websocket: Any) -> None:
        while True:
            await asyncio.sleep(1.0)
            await self._send(websocket, {
                'type': 'telemetry',
                'data': {
                    'x': self.x,
                    'y': self.y,
                    'yaw': self.yaw,
                    'battery': self.profile.battery_percent,
                    'battery_source': 'SIMULATED',
                    'frame_id': f"{self.profile.ros_namespace.lstrip('/')}/map",
                    'timestamp': utc_timestamp(),
                },
            })

    async def _send_readiness(self, websocket: Any) -> None:
        validation_results = [
            {
                'check_id': 'capability.fleet_lab_profile',
                'category': 'CAPABILITY',
                'status': 'PASS',
                'message': 'Fleet Lab profile is configured',
                'observed': self.profile.profile_version,
            },
            {
                'check_id': 'data.map',
                'category': 'DATA',
                'status': 'PASS',
                'message': 'Simulated active map is configured',
                'observed': self.profile.active_map_id,
            },
        ]
        await self._send(websocket, {
            'type': 'agent_readiness',
            'protocol_version': '1.0',
            'robot_id': self.robot_id,
            'status': 'READY',
            'checks': {'nav2': True, 'map': True, 'localization': True},
            'validation_results': validation_results,
            'active_map_id': self.profile.active_map_id,
            'detail': f'Fleet Lab ready in ROS namespace {self.profile.ros_namespace}',
            'timestamp': utc_timestamp(),
        })

    async def _handle_message(self, websocket: Any, message: dict[str, Any]) -> None:
        if message.get('type') == 'route_preview_request':
            await self._send_route_preview(websocket, message)
            return
        if message.get('type') != 'command' or message.get('command') != 'navigate_to_pose':
            return
        command_id = message.get('command_id')
        task_id = message.get('task_id')
        stage = message.get('stage')
        target = message.get('target', {})
        if not all(isinstance(value, str) and value for value in (command_id, task_id, stage)):
            return
        await self._send(websocket, {
            'type': 'command_status',
            'protocol_version': '1.0',
            'command_id': command_id,
            'robot_id': self.robot_id,
            'lifecycle': 'accepted',
            'timestamp': utc_timestamp(),
            'detail': 'Fleet Lab accepted navigation command',
        })
        await asyncio.sleep(self.profile.navigation_delay_seconds)
        if isinstance(target, dict):
            self.x = float(target.get('x', self.x))
            self.y = float(target.get('y', self.y))
            self.yaw = float(target.get('yaw', self.yaw))
        await self._send(websocket, {
            'type': 'navigation_result',
            'command_id': command_id,
            'task_id': task_id,
            'stage': stage,
            'status': 'succeeded',
            'detail': 'Fleet Lab simulated Nav2 success',
        })

    async def _send_route_preview(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        request_id = message.get('request_id')
        start = message.get('start')
        pickup = message.get('pickup')
        destination = message.get('destination')
        poses = (start, pickup, destination)
        if not isinstance(request_id, str) or not all(
            isinstance(pose, dict) for pose in poses
        ):
            return

        def point(pose: dict[str, Any]) -> dict[str, float]:
            return {
                'x': float(pose['x']),
                'y': float(pose['y']),
                'yaw': float(pose.get('yaw', 0.0)),
            }

        await self._send(websocket, {
            'type': 'route_preview_result',
            'request_id': request_id,
            'status': 'available',
            'frame_id': 'map',
            'pickup_path': [point(start), point(pickup)],
            'delivery_path': [point(pickup), point(destination)],
            'detail': 'Fleet Lab generated a straight-line route preview',
        })

    @staticmethod
    async def _send(websocket: Any, payload: dict[str, Any]) -> None:
        await websocket.send(json.dumps(payload))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--server-url', default='http://localhost:8000')
    parser.add_argument('--credential-file', required=True, type=Path)
    return parser.parse_args()


async def async_main() -> None:
    args = parse_args()
    simulator = FleetAgentSimulator(
        FleetAgentProfile.load(args.profile),
        args.server_url,
        args.credential_file,
        os.getenv('ROBOT_ENROLLMENT_TOKEN', ''),
    )
    loop = asyncio.get_running_loop()
    agent_task = asyncio.create_task(simulator.run())
    for signal_name in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signal_name, agent_task.cancel)
    try:
        await agent_task
    except asyncio.CancelledError:
        simulator.stop()


def main() -> None:
    """Run one Fleet Lab agent until interrupted."""
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
