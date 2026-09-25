"""Tests for the low-latency outbound camera relay."""

from amr_web_bridge.camera_relay import LatestJpegSource


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
    assert source.frame == second


def test_mjpeg_source_returns_new_frame_once_per_sequence():
    source = LatestJpegSource('http://127.0.0.1:8081/stream')
    with source.condition:
        source.frame = b'\xff\xd8frame\xff\xd9'
        source.sequence = 7

    assert source.wait_for_frame(6, 0) == (7, source.frame)
    assert source.wait_for_frame(7, 0) is None
