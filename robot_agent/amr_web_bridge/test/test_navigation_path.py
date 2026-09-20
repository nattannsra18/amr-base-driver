import threading
from types import SimpleNamespace

from amr_web_bridge.path_utils import (
    downsample_preserving_endpoints,
    serialize_path,
    serialize_preview_path,
)
from amr_web_bridge.web_bridge_node import WebBridgeNode


def pose(x, y, yaw_quaternion=(0.0, 1.0)):
    z, w = yaw_quaternion
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=z,
                w=w,
            ),
        )
    )


def path(poses):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='map',
            stamp=SimpleNamespace(sec=10, nanosec=0),
        ),
        poses=poses,
    )


COMMAND = {
    'command_id': 'Task-001:pickup:abc',
    'task_id': 'Task-001',
    'stage': 'pickup',
}


def test_downsampling_is_deterministic_and_preserves_endpoints():
    values = list(range(1000))
    first = downsample_preserving_endpoints(values, 500)
    second = downsample_preserving_endpoints(values, 500)
    assert first == second
    assert len(first) == 500
    assert first[0] == 0
    assert first[-1] == 999


def test_path_serialization_filters_non_finite_values_and_adds_yaw():
    message = path([
        pose(0.0, 1.0),
        pose(float('nan'), 2.0),
        pose(3.0, float('inf')),
        pose(4.0, 5.0, (float('nan'), 1.0)),
    ])
    serialized = serialize_path(
        message,
        COMMAND,
        500,
        '2026-09-03T10:00:00+00:00',
    )
    assert serialized is not None
    assert serialized['type'] == 'navigation_path'
    assert serialized['command_id'] == COMMAND['command_id']
    assert serialized['frame_id'] == 'map'
    assert serialized['poses'] == [
        {'x': 0.0, 'y': 1.0, 'yaw': 0.0},
        {'x': 4.0, 'y': 5.0},
    ]


def test_no_path_is_serialized_without_active_command():
    assert serialize_path(
        path([pose(1.0, 2.0)]),
        None,
        500,
        '2026-09-03T10:00:00+00:00',
    ) is None


def test_preview_path_serialization_needs_no_navigation_command():
    serialized = serialize_preview_path(
        path([pose(0.0, 0.0), pose(3.0, 4.0)]),
        500,
    )
    assert serialized == (
        'map',
        [
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 3.0, 'y': 4.0, 'yaw': 0.0},
        ],
    )


def make_bridge(command=None):
    bridge = SimpleNamespace(
        command_lock=threading.Lock(),
        path_lock=threading.Lock(),
        active_command=command,
        latest_path=None,
        latest_path_signature=None,
        path_revision=0,
        path_max_poses=500,
        sent=[],
    )
    bridge.message_timestamp = lambda _: '2026-09-03T10:00:00+00:00'
    bridge.send_from_ros = bridge.sent.append
    bridge.clear_navigation_path = lambda *args, **kwargs: (
        WebBridgeNode.clear_navigation_path(bridge, *args, **kwargs)
    )
    return bridge


def test_path_callback_associates_path_with_active_command():
    bridge = make_bridge(dict(COMMAND))
    message = path([pose(1.0, 2.0)])
    WebBridgeNode.path_callback(bridge, message)
    WebBridgeNode.path_callback(bridge, message)
    assert bridge.latest_path is not None
    assert bridge.latest_path['command_id'] == COMMAND['command_id']
    assert bridge.path_revision == 1


def test_path_callback_does_nothing_without_active_command():
    bridge = make_bridge()
    WebBridgeNode.path_callback(bridge, path([pose(1.0, 2.0)]))
    assert bridge.latest_path is None
    assert bridge.path_revision == 0


def test_empty_active_path_sends_explicit_clear_event():
    bridge = make_bridge(dict(COMMAND))
    bridge.latest_path = {'poses': [{'x': 1.0, 'y': 2.0}]}
    WebBridgeNode.path_callback(bridge, path([]))
    assert bridge.latest_path is None
    assert bridge.sent == [{
        'type': 'navigation_path_clear',
        'command_id': COMMAND['command_id'],
        'task_id': COMMAND['task_id'],
        'stage': COMMAND['stage'],
    }]


def test_clear_active_command_clears_local_path():
    bridge = make_bridge(dict(COMMAND))
    bridge.active_goal_handle = object()
    bridge.latest_path = {'poses': [{'x': 1.0, 'y': 2.0}]}
    WebBridgeNode.clear_active_command(
        bridge,
        COMMAND['command_id'],
    )
    assert bridge.active_command is None
    assert bridge.active_goal_handle is None
    assert bridge.latest_path is None
    assert bridge.sent == []
