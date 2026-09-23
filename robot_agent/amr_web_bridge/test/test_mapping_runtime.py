import json
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace

from amr_web_bridge.mapping_runtime import MappingRuntime
import yaml


class FakeProcess:
    pid = 12345

    def poll(self):
        return None

    def wait(self, timeout):
        return 0


def test_web_mapping_uses_detailed_physical_slam_profile():
    config_path = Path(__file__).parents[1] / 'config' / 'web_mapping.yaml'
    parameters = yaml.safe_load(config_path.read_text(encoding='utf-8'))[
        'slam_toolbox'
    ]['ros__parameters']

    assert parameters['use_sim_time'] is False
    assert parameters['resolution'] == 0.03
    assert parameters['scan_queue_size'] == 10
    assert parameters['minimum_travel_distance'] == 0.05
    assert parameters['minimum_travel_heading'] == 0.05
    assert parameters['max_laser_range'] == 8.0


def test_mapping_runtime_transitions_and_saves_validated_map(tmp_path, monkeypatch):
    calls = []
    launched = []

    def run(arguments, **_kwargs):
        calls.append(arguments)
        if any(value.endswith('/save_map') for value in arguments):
            target = Path(json.loads(arguments[-1])['name']['data'])
            target.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
            target.with_suffix('.yaml').write_text(
                f'image: {target.name}.pgm\nresolution: 0.05\n',
                encoding='utf-8',
            )
            output = 'response: slam_toolbox.srv.SaveMap_Response(result=0)'
        elif 'lifecycle' in arguments and 'get' in arguments:
            output = 'active [3]'
        else:
            output = 'response: nav2_msgs.srv.ManageLifecycleNodes_Response(success=True)'
        return SimpleNamespace(returncode=0, stdout=output, stderr='')

    monkeypatch.setattr('amr_web_bridge.mapping_runtime.os.killpg', lambda *_args: None)

    def popen(arguments, **_kwargs):
        launched.append(arguments)
        return FakeProcess()

    runtime = MappingRuntime(
        str(tmp_path),
        run=run,
        popen=popen,
        sleep=lambda _seconds: None,
    )
    runtime.start('mapping:robot01:abc')
    assert 'online_sync_launch.py' in launched[0]
    assert 'use_sim_time:=false' in launched[0]
    assert runtime.snapshot(2)['phase'] == 'MAPPING'
    runtime.stop_capture()
    assert runtime.snapshot(3)['phase'] == 'REVIEW'
    runtime.save('new_floor', {
        'name': 'New Floor', 'building': 'A', 'floor': '2', 'area_description': None,
    })
    snapshot = runtime.snapshot(4)
    assert snapshot['phase'] == 'IDLE'
    assert snapshot['saved_map_id'] == 'new_floor'
    assert (tmp_path / 'new_floor.metadata.json').is_file()
    assert sum(
        any(value.endswith('/manage_nodes') for value in command)
        for command in calls
    ) == 4


def test_mapping_runtime_rejects_existing_map(tmp_path):
    (tmp_path / 'used.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    runtime = MappingRuntime(str(tmp_path), sleep=lambda _seconds: None)
    runtime._phase = 'REVIEW'
    try:
        runtime.save('used', {'name': 'Used'})
    except ValueError as error:
        assert str(error) == 'Map ID already exists'
    else:
        raise AssertionError('existing map ID was accepted')
    assert runtime.snapshot(1)['phase'] == 'REVIEW'


def test_mapping_runtime_saves_exact_live_grid_and_activates_it(
    tmp_path, monkeypatch,
):
    activated = []
    monkeypatch.setattr(
        'amr_web_bridge.mapping_runtime.os.killpg', lambda *_args: None,
    )
    runtime = MappingRuntime(str(tmp_path), sleep=lambda _seconds: None)
    runtime._phase = 'REVIEW'
    runtime._process = FakeProcess()
    runtime._start_map_revision = 7

    runtime.save(
        'fresh_map',
        {'name': 'Fresh map'},
        {
            'width': 2,
            'height': 2,
            'resolution': 0.05,
            'origin_x': -1.0,
            'origin_y': 2.0,
            'origin_yaw': 0.25,
            'data': [0, 100, -1, 50],
            'revision': 8,
        },
        lambda path: activated.append(path),
    )

    assert activated == [tmp_path / 'fresh_map.yaml']
    assert (tmp_path / 'fresh_map.pgm').read_bytes() == (
        b'P5\n2 2\n255\n' + bytes([205, 205, 254, 0])
    )
    saved = yaml.safe_load(
        (tmp_path / 'fresh_map.yaml').read_text(encoding='utf-8')
    )
    assert saved['image'] == 'fresh_map.pgm'
    assert saved['origin'] == [-1.0, 2.0, 0.25]
    assert runtime.snapshot(9)['phase'] == 'IDLE'
    assert runtime.snapshot(9)['saved_map_id'] == 'fresh_map'


def test_mapping_runtime_rejects_stale_grid_from_before_session(tmp_path):
    runtime = MappingRuntime(str(tmp_path), sleep=lambda _seconds: None)
    runtime._phase = 'REVIEW'
    runtime._start_map_revision = 9

    try:
        runtime.save(
            'stale_map',
            {'name': 'Stale map'},
            {
                'width': 1,
                'height': 1,
                'resolution': 0.05,
                'data': [0],
                'revision': 9,
            },
        )
    except RuntimeError as error:
        assert str(error) == 'Waiting for the first map from this SLAM session'
    else:
        raise AssertionError('stale pre-session map was saved')
    assert runtime.snapshot(9)['phase'] == 'REVIEW'


def test_mapping_runtime_preserves_start_failure_detail(tmp_path):
    def run(_arguments, **_kwargs):
        raise RuntimeError('localization lifecycle service timed out')

    runtime = MappingRuntime(
        str(tmp_path),
        run=run,
        sleep=lambda _seconds: None,
    )
    try:
        runtime.start('mapping:robot01:failed')
    except RuntimeError as error:
        assert str(error) == 'localization lifecycle service timed out'
    else:
        raise AssertionError('mapping start failure was not raised')

    snapshot = runtime.snapshot(1)
    assert snapshot['phase'] == 'FAILED'
    assert snapshot['detail'] == (
        'Unable to enter ROS mapping mode: '
        'localization lifecycle service timed out'
    )


def test_mapping_runtime_limits_lifecycle_discovery_and_reports_timeout(tmp_path):
    def run(_arguments, **_kwargs):
        raise subprocess.TimeoutExpired('ros2 lifecycle get /amcl', 12.0)

    runtime = MappingRuntime(
        str(tmp_path),
        run=run,
        sleep=lambda _seconds: None,
    )
    started = time.monotonic()
    try:
        runtime.start('mapping:robot01:timeout')
    except RuntimeError as error:
        assert 'timed out after 12s' in str(error)
        assert 'ros2 lifecycle get' in str(error)
    else:
        raise AssertionError('lifecycle timeout was not reported')

    assert time.monotonic() - started < 1.0
    assert runtime.snapshot(1)['detail'].startswith(
        'Unable to enter ROS mapping mode: ROS command timed out after 12s'
    )


def test_mapping_skips_navigation_when_waiting_for_initial_pose(
    tmp_path, monkeypatch,
):
    calls = []

    def run(arguments, **_kwargs):
        calls.append(arguments)
        if 'lifecycle' in arguments and 'get' in arguments:
            output = (
                'unconfigured [1]'
                if arguments[-1] == '/planner_server'
                else 'active [3]'
            )
        else:
            output = (
                'response: '
                'nav2_msgs.srv.ManageLifecycleNodes_Response(success=True)'
            )
        return SimpleNamespace(returncode=0, stdout=output, stderr='')

    monkeypatch.setattr(
        'amr_web_bridge.mapping_runtime.os.killpg', lambda *_args: None,
    )
    runtime = MappingRuntime(
        str(tmp_path),
        run=run,
        popen=lambda *_args, **_kwargs: FakeProcess(),
        sleep=lambda _seconds: None,
    )
    runtime.start('mapping:robot01:no-initial-pose')
    runtime.discard()

    manager_calls = [
        command for command in calls
        if any(value.endswith('/manage_nodes') for value in command)
    ]
    assert len(manager_calls) == 2
    assert all(
        '/lifecycle_manager_localization/manage_nodes' in command
        for command in manager_calls
    )


def test_discard_accepts_lifecycle_started_after_lost_response(
    tmp_path, monkeypatch,
):
    calls = []

    def run(arguments, **_kwargs):
        calls.append(arguments)
        if 'service' in arguments and 'call' in arguments:
            raise RuntimeError('lifecycle response timed out')
        if arguments[-1] == '/amcl':
            return SimpleNamespace(
                returncode=0, stdout='active [3]', stderr='',
            )
        return SimpleNamespace(
            returncode=0, stdout='unconfigured [1]', stderr='',
        )

    monkeypatch.setattr(
        'amr_web_bridge.mapping_runtime.os.killpg', lambda *_args: None,
    )
    runtime = MappingRuntime(str(tmp_path), run=run, sleep=lambda _seconds: None)
    runtime._phase = 'FAILED'
    runtime._localization_was_active = True
    runtime.discard()

    assert runtime.snapshot(1)['phase'] == 'IDLE'
    assert sum('service' in command for command in calls) == 1


def test_discard_reports_restore_failure_and_remains_retryable(tmp_path):
    def run(_arguments, **_kwargs):
        raise RuntimeError('DDS service unavailable')

    runtime = MappingRuntime(str(tmp_path), run=run, sleep=lambda _seconds: None)
    runtime._phase = 'FAILED'
    runtime._localization_was_active = True

    try:
        runtime.discard()
    except RuntimeError as error:
        assert 'failed to restore amcl after 3 attempts' in str(error)
    else:
        raise AssertionError('restore failure was not raised')

    snapshot = runtime.snapshot(1)
    assert snapshot['phase'] == 'FAILED'
    assert 'Unable to restore Nav2 localization' in snapshot['detail']
    assert runtime._localization_was_active is True
