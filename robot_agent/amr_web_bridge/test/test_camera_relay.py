"""Tests for the low-latency outbound camera relay."""

import struct

from amr_web_bridge.camera_relay import (
    camera_frame_acknowledged,
    CAMERA_FRAME_HEADER,
    CAMERA_FRAME_MAGIC,
    CAMERA_FRAME_VERSION,
    CapturedJpeg,
    LatestJpegSource,
    pack_camera_frame,
)


class ChunkStream:
    def __init__(self, chunks):
        self.chunks = iter(chunks)

    def read(self, _size):
        return next(self.chunks, b'')


def test_mjpeg_source_keeps_only_the_latest_complete_frame():
    source = LatestJpegSource('http://127.0.0.1:8081/stream')
    first = b'\xff\xd8first\xff\xd9'
    second = b'\xff\xd8second\xff\xd9'
    stream = ChunkStream([
        b'headers' + first[:5],
        first[5:] + b'boundary' + second,
    ])

    try:
        source._read_stream(stream)
    except OSError as error:
        assert str(error) == 'camera stream ended'

    assert source.sequence == 2
    assert source.frame is not None
    assert source.frame.data == second
    assert source.frame.sequence == 2
    assert source.frame.captured_at_ns > 0


def test_mjpeg_source_returns_new_frame_once_per_sequence():
    source = LatestJpegSource('http://127.0.0.1:8081/stream')
    with source.condition:
        source.frame = CapturedJpeg(
            sequence=7,
            captured_at_ns=123,
            data=b'\xff\xd8frame\xff\xd9',
        )
        source.sequence = 7

    assert source.wait_for_frame(6, 0) == source.frame
    assert source.wait_for_frame(7, 0) is None


def test_camera_frame_envelope_carries_source_timestamps_without_base64():
    jpeg = b'\xff\xd8frame\xff\xd9'
    payload = pack_camera_frame(
        CapturedJpeg(sequence=9, captured_at_ns=123, data=jpeg),
        sent_at_ns=456,
    )

    magic, version, sequence, captured_at_ns, sent_at_ns = (
        CAMERA_FRAME_HEADER.unpack_from(payload)
    )
    assert magic == CAMERA_FRAME_MAGIC
    assert version == CAMERA_FRAME_VERSION
    assert sequence == 9
    assert captured_at_ns == 123
    assert sent_at_ns == 456
    assert payload[CAMERA_FRAME_HEADER.size:] == jpeg
    assert CAMERA_FRAME_HEADER.size == struct.calcsize('!4sBQQQ')


def test_camera_frame_ack_must_match_the_one_frame_in_flight():
    assert camera_frame_acknowledged(
        '{"type":"camera_frame_ack","source_sequence":9}',
        9,
    )
    assert not camera_frame_acknowledged(
        '{"type":"camera_frame_ack","source_sequence":8}',
        9,
    )
    assert not camera_frame_acknowledged('not-json', 9)
