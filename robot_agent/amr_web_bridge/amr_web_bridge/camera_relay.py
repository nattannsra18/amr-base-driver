"""Relay the robot's local MJPEG camera to the control plane over WSS."""

from __future__ import annotations

import asyncio
import json
import logging
import os
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from urllib.request import urlopen

import websockets

from .agent_identity import AgentCredentialStore


LOGGER = logging.getLogger('amr_camera_relay')
MAX_BUFFER_BYTES = 2_000_000
CAMERA_FRAME_MAGIC = b'IDRC'
CAMERA_FRAME_VERSION = 1
CAMERA_FRAME_HEADER = struct.Struct('!4sBQQQ')
CAMERA_MAX_IN_FLIGHT = 2


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


def camera_frame_acknowledged(message: object, sequence: int) -> bool:
    """Validate the one-frame-in-flight acknowledgement from FastAPI."""
    try:
        payload = json.loads(str(message))
    except (TypeError, ValueError):
        return False
    return (
        payload.get('type') == 'camera_frame_ack'
        and payload.get('source_sequence') == sequence
    )


class LatestJpegSource:
    """Continuously decode an MJPEG byte stream while retaining one frame."""

    def __init__(self, url: str):
        self.url = url
        self.condition = threading.Condition()
        self.frame: CapturedJpeg | None = None
        self.sequence = 0
        self.stopped = threading.Event()
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if self.thread is not None:
            return
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stopped.set()
        with self.condition:
            self.condition.notify_all()

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
        while not self.stopped.is_set():
            chunk = stream.read(16_384)
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
                with self.condition:
                    self.sequence += 1
                    self.frame = CapturedJpeg(
                        sequence=self.sequence,
                        captured_at_ns=time.time_ns(),
                        data=frame,
                    )
                    self.condition.notify_all()


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
                    f'{credential.robot_id}/camera'
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
            sequence = 0
            next_send = 0.0
            pending_sequences: deque[int] = deque()
            while True:
                if len(pending_sequences) < CAMERA_MAX_IN_FLIGHT:
                    result = await asyncio.to_thread(
                        self.source.wait_for_frame,
                        sequence,
                        5.0 if not pending_sequences else self.frame_interval,
                    )
                    if result is not None:
                        sequence = result.sequence
                        frame = result
                        delay = next_send - time.monotonic()
                        if delay > 0:
                            await asyncio.sleep(delay)
                            newer = await asyncio.to_thread(
                                self.source.wait_for_frame,
                                sequence,
                                0.0,
                            )
                            if newer is not None:
                                sequence = newer.sequence
                                frame = newer
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
                    timeout=3.0,
                )
                if not camera_frame_acknowledged(
                    acknowledgement,
                    pending_sequences[0],
                ):
                    raise OSError('invalid camera frame acknowledgement')
                pending_sequences.popleft()


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
