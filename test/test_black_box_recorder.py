import json
import math
from types import SimpleNamespace

from amr_base_driver.black_box_recorder import (
    aborted_goal_ids,
    BlackBoxRecorder,
    compact_path,
    diagnostic_level,
    RollingJsonlWriter,
    sector_clearances,
)


def test_compact_path_preserves_endpoints_and_stays_bounded():
    def pose(index):
        return SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=index, y=-index),
                orientation=SimpleNamespace(z=0.0, w=1.0),
            ))

    message = SimpleNamespace(poses=[pose(index) for index in range(101)])

    result = compact_path(message, max_poses=5)

    assert len(result) == 5
    assert result[0]['x'] == 0.0
    assert result[-1]['x'] == 100.0


def test_aborted_goal_ids_keeps_only_terminal_aborts():
    aborted = SimpleNamespace(
        status=6,
        goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=b'\x01' * 16)),
    )
    active = SimpleNamespace(
        status=2,
        goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=b'\x02' * 16)),
    )

    assert aborted_goal_ids(SimpleNamespace(status_list=[active, aborted])) == [
        '01' * 16
    ]


def test_navigation_abort_archives_each_goal_once():
    recorder = BlackBoxRecorder.__new__(BlackBoxRecorder)
    recorder.archived_abort_goal_ids = set()
    archived = []
    recorder.archive_incident = lambda label, marker: archived.append(
        (label, marker))
    aborted = SimpleNamespace(
        status=6,
        goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=b'\x01' * 16)),
    )
    message = SimpleNamespace(status_list=[aborted])

    recorder.record_navigation_status(message)
    recorder.record_navigation_status(message)

    assert archived == [
        ('nav2-aborted', {'event': 'nav2_aborted', 'goal_id': '01' * 16})
    ]


def test_navigation_success_archives_task_stage():
    recorder = BlackBoxRecorder.__new__(BlackBoxRecorder)
    recorded = []
    archived = []
    recorder.record = lambda stream, data, force=False: recorded.append(
        (stream, data, force))
    recorder.archive_incident = lambda label, marker: archived.append(
        (label, marker))
    event = {
        'event': 'navigation_succeeded',
        'task_id': 'TASK-309',
        'stage': 'destination',
    }

    recorder.record_navigation_event(SimpleNamespace(data=json.dumps(event)))

    assert recorded == [('/navigation_events', event, True)]
    assert archived == [('navigation_succeeded-TASK-309-destination', event)]


def test_diagnostic_fault_bypasses_continuous_stream_throttle():
    recorder = BlackBoxRecorder.__new__(BlackBoxRecorder)
    recorder.should_record = lambda _stream: False
    recorded = []
    recorder.record = lambda stream, data, force=False: recorded.append(
        (stream, data, force))
    recorder.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=0))
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        status=[SimpleNamespace(
            name='Base serial telemetry',
            level=2,
            message='Telemetry stale',
            values=[],
        )],
    )

    recorder.record_diagnostics(message)

    assert recorded[0][0] == '/diagnostics/Base serial telemetry'
    assert recorded[0][2] is True


def test_throttled_ok_diagnostics_skip_value_materialization():
    recorder = BlackBoxRecorder.__new__(BlackBoxRecorder)
    recorder.should_record = lambda _stream: False
    recorder.record = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError('throttled diagnostics must not be written'))

    class ValuesThatMustNotBeRead:
        def __iter__(self):
            raise AssertionError('diagnostic values were materialized')

    message = SimpleNamespace(status=[SimpleNamespace(
        name='Base drive telemetry',
        level=0,
        message='OK',
        values=ValuesThatMustNotBeRead(),
    )])

    recorder.record_diagnostics(message)


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
