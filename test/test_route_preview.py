import asyncio
import inspect
from queue import Queue
import threading
from types import SimpleNamespace

from amr_web_bridge.web_bridge_node import WebBridgeNode


def preview_request():
    return {
        'type': 'route_preview_request',
        'request_id': 'preview-1',
        'start': {'frame_id': 'map', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'pickup': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
        'destination': {'frame_id': 'map', 'x': 3.0, 'y': 4.0, 'yaw': 1.57},
    }


def bridge():
    value = SimpleNamespace(
        preview_queue=Queue(),
        preview_lock=threading.Lock(),
        map_command_queue=Queue(),
        map_command_lock=threading.Lock(),
        active_preview=None,
        active_map_command=None,
        sent=[],
    )

    async def send_json(_websocket, message):
        value.sent.append(message)

    value.send_json = send_json
    value.send_from_ros = value.sent.append
    value.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None,
    )
    return value


def test_valid_preview_is_queued_without_navigation_goal():
    value = bridge()
    request = preview_request()
    asyncio.run(WebBridgeNode.queue_route_preview(value, object(), request))
    assert value.preview_queue.get_nowait() == request
    assert value.sent == []


def test_invalid_preview_returns_unavailable_result():
    value = bridge()
    request = preview_request()
    request['pickup']['x'] = float('nan')
    asyncio.run(WebBridgeNode.queue_route_preview(value, object(), request))
    assert value.preview_queue.empty()
    assert value.sent[-1]['type'] == 'route_preview_result'
    assert value.sent[-1]['status'] == 'unavailable'


def test_preview_is_rejected_while_map_switch_is_active():
    value = bridge()
    value.active_map_command = {'command_id': 'map-switch:robot01:abc'}

    asyncio.run(
        WebBridgeNode.queue_route_preview(
            value,
            object(),
            preview_request(),
        )
    )

    assert value.preview_queue.empty()
    assert value.sent[-1]['status'] == 'unavailable'
    assert value.sent[-1]['detail'] == 'Map switch is in progress'


def test_finishing_preview_clears_state_and_returns_both_paths():
    value = bridge()
    request = preview_request()
    value.active_preview = {'request': request}
    WebBridgeNode.finish_route_preview(
        value,
        request,
        'available',
        'reachable',
        frame_id='map',
        pickup_path=[{'x': 0.0, 'y': 0.0}],
        delivery_path=[{'x': 1.0, 'y': 1.0}],
    )
    assert value.active_preview is None
    assert value.sent[-1]['status'] == 'available'
    assert value.sent[-1]['pickup_path'] == [{'x': 0.0, 'y': 0.0}]
    assert value.sent[-1]['delivery_path'] == [{'x': 1.0, 'y': 1.0}]


def test_preview_goal_uses_compute_path_and_never_navigate_to_pose():
    source = inspect.getsource(WebBridgeNode.build_preview_goal)
    assert 'ComputePathToPose.Goal' in source
    assert 'NavigateToPose' not in source
