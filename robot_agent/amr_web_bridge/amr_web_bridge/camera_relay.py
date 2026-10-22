"""Relay the robot's local MJPEG camera to the control plane over WSS."""

from __future__ import annotations

import asyncio
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
import json
import logging
import os
from pathlib import Path
import struct
import threading
import time
from urllib.request import urlopen

import websockets

from .agent_identity import AgentCredentialStore


LOGGER = logging.getLogger('amr_camera_relay')
MAX_BUFFER_BYTES = 2_000_000
CAMERA_FRAME_MAGIC = b'IDRC'
CAMERA_FRAME_VERSION = 1
CAMERA_FRAME_HEADER = struct.Struct('!4sBQQQ')
# Three frames cover the measured Wi-Fi RTT at 30 FPS while bounding wire
# backlog to roughly 100 ms during a brief network stall.
CAMERA_MAX_IN_FLIGHT = 3
CAMERA_ACK_TIMEOUT_SECONDS = 8.0


@dataclass(frozen=True)
class CapturedJpeg:
    """One completed JPEG and its source-side capture metadata."""

    sequence: int
    captured_at_ns: int
    data: bytes


def pack_camera_frame(frame: CapturedJpeg, sent_at_ns: int) -> bytes:
    """Add compact timing metadata without base64 or an extra message."""
    return CAMERA_FRAME_HEADER.pack(
        CAMERA_FRAME_MAGIC,
        CAMERA_FRAME_VERSION,
        frame.sequence,
        frame.captured_at_ns,
        sent_at_ns,
    ) + frame.data


def camera_frame_ack_sequence(message: object) -> int | None:
    """Return a valid FastAPI camera acknowledgement sequence."""
    try:
        payload = json.loads(str(message))
    except (TypeError, ValueError):
        return None
    sequence = payload.get('source_sequence')
    if payload.get('type') != 'camera_frame_ack':
        return None
    return sequence if isinstance(sequence, int) and sequence >= 0 else None


def camera_frame_acknowledged(message: object, sequence: int) -> bool:
    """Validate a camera acknowledgement from FastAPI."""
    return camera_frame_ack_sequence(message) == sequence


def camera_stream_requested(message: object) -> bool | None:
    """Return a server demand update, or None for another message type."""
    try:
        payload = json.loads(str(message))
    except (TypeError, ValueError):
        return None
    if payload.get('type') not in {'camera_ready', 'camera_demand'}:
        return None
    requested = payload.get('stream_requested')
    return requested if isinstance(requested, bool) else None


class LatestJpegSource:
    """Continuously decode an MJPEG byte stream while retaining one frame."""

    def __init__(self, url: str):
        self.url = url
        self.condition = threading.Condition()
        self.frame: CapturedJpeg | None = None
        self.sequence = 0
        self.stopped = threading.Event()
        self.active = threading.Event()
        self.active.set()
        self.thread: threading.Thread | None = None
        self.frame_callback: Callable[[CapturedJpeg], None] | None = None

    def start(self) -> None:
        if self.thread is not None:
            return
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stopped.set()
        self.active.set()
        with self.condition:
            self.condition.notify_all()

    def set_active(self, active: bool) -> None:
        if active:
            self.active.set()
        else:
            self.active.clear()
        with self.condition:
            self.condition.notify_all()

    def set_frame_callback(
        self,
        callback: Callable[[CapturedJpeg], None] | None,
    ) -> None:
        self.frame_callback = callback

    def wait_for_frame(
        self,
        after_sequence: int,
        timeout: float,
    ) -> CapturedJpeg | None:
        with self.condition:
            self.condition.wait_for(
                lambda: self.stopped.is_set()
                or (self.frame is not None and self.sequence > after_sequence),
                timeout=timeout,
            )
            if self.frame is None or self.sequence <= after_sequence:
                return None
            return self.frame

    def _run(self) -> None:
        while not self.stopped.is_set():
            if not self.active.wait(timeout=1.0):
                continue
            try:
                with urlopen(self.url, timeout=10) as stream:
                    LOGGER.info('Reading local camera stream: %s', self.url)
                    self._read_stream(stream)
            except OSError as error:
                if not self.stopped.is_set():
                    LOGGER.warning('Local camera unavailable: %s', error)
                    self.stopped.wait(2.0)

    def _read_stream(self, stream) -> None:
        buffer = bytearray()
        # HTTPResponse.read(size) waits for the full size, which batches
        # several small JPEGs and collapses a 30 FPS source to single digits.
        # read1() returns currently available socket data without that delay.
        read_available = getattr(stream, 'read1', stream.read)
        while not self.stopped.is_set() and self.active.is_set():
            # One 4 KiB read closely matches this camera's native MJPEG frame
            # size, avoiding coalescing several frames into one notification.
            chunk = read_available(4_096)
            if not chunk:
                raise OSError('camera stream ended')
            buffer.extend(chunk)
            while True:
                start = buffer.find(b'\xff\xd8')
                if start < 0:
                    if len(buffer) > MAX_BUFFER_BYTES:
                        del buffer[:-2]
                    break
                end = buffer.find(b'\xff\xd9', start + 2)
                if end < 0:
                    if start > 0:
                        del buffer[:start]
                    if len(buffer) > MAX_BUFFER_BYTES:
                        del buffer[:-2]
                    break
                frame = bytes(buffer[start:end + 2])
                del buffer[:end + 2]
                captured = CapturedJpeg(
                    sequence=self.sequence + 1,
                    captured_at_ns=time.time_ns(),
                    data=frame,
                )
                with self.condition:
                    self.sequence += 1
                    self.frame = captured
                    self.condition.notify_all()
                callback = self.frame_callback
                if callback is not None:
                    callback(captured)


class CameraRelay:
    """Publish latest local JPEGs on a dedicated authenticated WSS channel."""

    def __init__(
        self,
        server_url: str,
        credential_file: Path,
        camera_url: str,
        max_fps: float,
    ):
        self.server_url = server_url.rstrip('/')
        self.credential_store = AgentCredentialStore(credential_file)
        self.source = LatestJpegSource(camera_url)
        self.frame_interval = 1.0 / max(1.0, min(max_fps, 30.0))

    async def run(self) -> None:
        self.source.set_active(False)
        self.source.start()
        try:
            while True:
                credential = self.credential_store.load()
                if credential is None:
                    LOGGER.warning('Waiting for the Robot Agent credential')
                    await asyncio.sleep(3.0)
                    continue
                uri = (
                    f'{self.server_url}/ws/robots/'
                    f'{credential.robot_id}/camera?demand_control=1'
                )
                try:
                    await self._publish(uri, credential.credential)
                except (OSError, asyncio.TimeoutError,
                        websockets.WebSocketException) as error:
                    LOGGER.warning('Camera relay disconnected: %s', error)
                    await asyncio.sleep(3.0)
        finally:
            self.source.stop()

    async def _publish(self, uri: str, credential: str) -> None:
        async with websockets.connect(
            uri,
            extra_headers={'Authorization': f'Bearer {credential}'},
            open_timeout=8,
            ping_interval=20,
            ping_timeout=20,
            max_size=64_000,
            compression=None,
        ) as websocket:
            ready = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            if 'camera_ready' not in str(ready):
                raise OSError('control plane rejected the camera handshake')
            LOGGER.info('Camera relay connected: %s', uri)
            stream_requested = camera_stream_requested(ready)
            # Old control planes do not advertise demand control. Preserve
            # their always-on behavior during rolling upgrades.
            if stream_requested is None:
                stream_requested = True
            self.source.set_active(stream_requested)
            sequence = 0
            next_send = 0.0
            pending_sequences: deque[int] = deque()
            frames: asyncio.Queue[CapturedJpeg] = asyncio.Queue(maxsize=1)
            loop = asyncio.get_running_loop()

            def enqueue_latest(frame: CapturedJpeg) -> None:
                def offer() -> None:
                    if frames.full():
                        frames.get_nowait()
                    frames.put_nowait(frame)

                loop.call_soon_threadsafe(offer)

            self.source.set_frame_callback(enqueue_latest)
            try:
                while True:
                    if not stream_requested:
                        control = await websocket.recv()
                        demand = camera_stream_requested(control)
                        if demand is not None:
                            stream_requested = demand
                            self.source.set_active(demand)
                        continue
                    if len(pending_sequences) < CAMERA_MAX_IN_FLIGHT:
                        try:
                            frame = await asyncio.wait_for(
                                frames.get(),
                                timeout=(
                                    5.0
                                    if not pending_sequences
                                    else self.frame_interval
                                ),
                            )
                        except asyncio.TimeoutError:
                            frame = None
                        if frame is not None:
                            sequence = frame.sequence
                            delay = next_send - time.monotonic()
                            if delay > 0:
                                await asyncio.sleep(delay)
                                while not frames.empty():
                                    frame = frames.get_nowait()
                                    sequence = frame.sequence
                            send_started = time.monotonic()
                            await websocket.send(
                                pack_camera_frame(frame, time.time_ns())
                            )
                            pending_sequences.append(sequence)
                            next_send = send_started + self.frame_interval
                            continue
                    if not pending_sequences:
                        continue
                    acknowledgement = await asyncio.wait_for(
                        websocket.recv(),
                        timeout=CAMERA_ACK_TIMEOUT_SECONDS,
                    )
                    demand = camera_stream_requested(acknowledgement)
                    if demand is not None:
                        stream_requested = demand
                        self.source.set_active(demand)
                        if not demand:
                            pending_sequences.clear()
                            while not frames.empty():
                                frames.get_nowait()
                        continue
                    acknowledged_sequence = camera_frame_ack_sequence(
                        acknowledgement
                    )
                    if acknowledged_sequence is None:
                        raise OSError('invalid camera frame acknowledgement')
                    # A demand update can overtake an ACK after its pending
                    # window is cleared. Ignore that stale ACK, and treat a
                    # newer ACK as cumulative because WebSocket delivery is
                    # ordered.
                    while (
                        pending_sequences
                        and pending_sequences[0] <= acknowledged_sequence
                    ):
                        pending_sequences.popleft()
            finally:
                self.source.set_frame_callback(None)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    server_url = os.getenv('ROBOT_CONTROL_URL', '').strip()
    credential_file = os.getenv('ROBOT_CREDENTIAL_FILE', '').strip()
    camera_url = os.getenv(
        'CAMERA_LOCAL_STREAM_URL',
        'http://127.0.0.1:8081/stream',
    ).strip()
    if not server_url or not credential_file:
        raise SystemExit(
            'ROBOT_CONTROL_URL and ROBOT_CREDENTIAL_FILE are required'
        )
    relay = CameraRelay(
        server_url,
        Path(credential_file),
        camera_url,
        float(os.getenv('CAMERA_RELAY_FPS', '30')),
    )
    try:
        asyncio.run(relay.run())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
