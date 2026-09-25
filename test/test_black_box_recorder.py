import json
import math
from types import SimpleNamespace

from amr_base_driver.black_box_recorder import (
    diagnostic_level,
    RollingJsonlWriter,
    sector_clearances,
)


def test_rolling_writer_keeps_only_retention_segments(tmp_path):
    writer = RollingJsonlWriter(
        tmp_path, segment_seconds=10.0, max_segments=3)
    for index, now in enumerate((0.0, 10.0, 20.0, 30.0)):
        writer.write({'index': index}, monotonic=now)
    writer.close()

    paths = writer.paths()
    assert len(paths) == 3
    events = [
        json.loads(line)
        for path in paths
        for line in path.read_text(encoding='utf-8').splitlines()
    ]
    assert [event['index'] for event in events] == [1, 2, 3]


def test_archive_preserves_the_current_rolling_window(tmp_path):
    rolling = tmp_path / 'rolling'
    writer = RollingJsonlWriter(
        rolling, segment_seconds=10.0, max_segments=3)
    writer.write({'index': 1}, monotonic=0.0)
    writer.write({'index': 2}, monotonic=10.0)

    archive = writer.archive(tmp_path / 'incidents', label='hil')
    writer.close()

    assert archive.name.startswith('hil-')
    events = [
        json.loads(line)
        for path in sorted(archive.glob('*.jsonl'))
        for line in path.read_text(encoding='utf-8').splitlines()
    ]
    assert [event['index'] for event in events] == [1, 2]


def test_sector_clearances_reject_invalid_ranges():
    scan = SimpleNamespace(
        ranges=[0.8, math.inf, 0.6, math.nan],
        angle_min=0.0,
        angle_increment=math.pi / 2.0,
        range_min=0.1,
        range_max=5.0,
    )

    assert sector_clearances(scan) == {
        'front': 0.8,
        'left': None,
        'right': None,
        'rear': 0.6,
    }


def test_diagnostic_level_accepts_jazzy_byte_and_integer_values():
    assert diagnostic_level(b'\x00') == 0
    assert diagnostic_level(bytearray([1])) == 1
    assert diagnostic_level(memoryview(b'\x02')) == 2
    assert diagnostic_level(3) == 3
    assert diagnostic_level(b'') == 3
