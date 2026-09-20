import json
from pathlib import Path
from types import SimpleNamespace

from amr_web_bridge.mapping_runtime import MappingRuntime


class FakeProcess:
    pid = 12345

    def poll(self):
        return None

    def wait(self, timeout):
        return 0


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
