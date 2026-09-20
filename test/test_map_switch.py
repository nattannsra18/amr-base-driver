import asyncio
from queue import Queue
import threading
from types import SimpleNamespace

from amr_web_bridge.web_bridge_node import WebBridgeNode
from nav2_msgs.srv import LoadMap


class ImmediateFuture:
    def __init__(self, response):
        self.response = response

    def result(self):
        return self.response

    def add_done_callback(self, callback):
        callback(self)


class LoadMapClient:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return timeout_sec == 1.0

    def call_async(self, request):
        self.requests.append(request)
        return ImmediateFuture(self.response)


def bridge(tmp_path):
    (tmp_path / 'second.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (tmp_path / 'second.yaml').write_text(
        'image: second.pgm\nresolution: 0.05\n',
        encoding='utf-8',
    )
    response = SimpleNamespace(
        result=LoadMap.Response.RESULT_SUCCESS,
        map=SimpleNamespace(info=SimpleNamespace(width=0, height=0)),
    )
    value = SimpleNamespace(
        robot_id='robot01',
        maps_directory=str(tmp_path),
        active_map_id='warehouse_map',
        command_lock=threading.Lock(),
        preview_lock=threading.Lock(),
        map_command_lock=threading.Lock(),
        active_command=None,
        active_preview=None,
        preview_queue=Queue(),
        active_map_command=None,
        map_command_queue=Queue(),
        load_map_client=LoadMapClient(response),
        sent=[],
    )
    value.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    value.send_from_ros = value.sent.append
    value.agent_readiness_snapshot = lambda: {
        'type': 'agent_readiness',
        'active_map_id': value.active_map_id,
    }
    value.finish_map_switch = lambda *args: WebBridgeNode.finish_map_switch(
        value, *args
    )
    value.map_switch_result_callback = (
        lambda *args: WebBridgeNode.map_switch_result_callback(value, *args)
    )

    async def send_json(_websocket, message):
        value.sent.append(message)

    value.send_json = send_json

    async def send_map_catalog(_websocket):
        value.sent.append({'type': 'map_catalog'})
    value.send_map_catalog = send_map_catalog
    return value


def command(map_id='second'):
    return {
        'type': 'map_command',
        'command': 'switch_map',
        'command_id': 'map-switch:robot01:abc',
        'robot_id': 'robot01',
        'map_id': map_id,
    }


def test_map_switch_uses_validated_robot_file_and_nav2_ack(tmp_path):
    value = bridge(tmp_path)
    asyncio.run(WebBridgeNode.handle_map_command(value, object(), command()))
    WebBridgeNode.process_map_command_queue(value)

    assert value.load_map_client.requests[0].map_url == str(
        tmp_path / 'second.yaml'
    )
    assert value.active_map_id == 'second'
    assert value.sent[0]['type'] == 'map_switch_result'
    assert value.sent[0]['accepted'] is True
    assert value.sent[1]['type'] == 'map_catalog'
    assert value.sent[1]['active_map_id'] == 'second'
    assert value.sent[2] == {
        'type': 'agent_readiness',
        'active_map_id': 'second',
    }


def test_map_switch_rejects_unavailable_map_before_nav2(tmp_path):
    value = bridge(tmp_path)
    asyncio.run(
        WebBridgeNode.handle_map_command(
            value,
            object(),
            command('../outside'),
        )
    )

    assert value.map_command_queue.empty()
    assert value.sent[-1]['accepted'] is False
    assert value.sent[-1]['detail'] == 'Map is unavailable on the robot'


def test_map_catalog_command_updates_metadata_and_reports_catalog(tmp_path):
    value = bridge(tmp_path)
    asyncio.run(WebBridgeNode.handle_map_catalog_command(value, object(), {
        'type': 'map_catalog_command',
        'command_id': 'map-catalog:robot01:abc',
        'robot_id': 'robot01',
        'map_id': 'second',
        'action': 'UPDATE_METADATA',
        'metadata': {
            'name': 'Second Floor',
            'building': 'Building A',
            'floor': '2',
            'area_description': None,
        },
    }))
    assert value.sent[-2]['type'] == 'map_catalog_operation_result'
    assert value.sent[-2]['accepted'] is True
    assert value.sent[-1]['type'] == 'map_catalog'
    assert (tmp_path / 'second.metadata.json').is_file()


def test_map_catalog_command_rejects_active_map_delete(tmp_path):
    value = bridge(tmp_path)
    value.active_map_id = 'second'
    asyncio.run(WebBridgeNode.handle_map_catalog_command(value, object(), {
        'type': 'map_catalog_command',
        'command_id': 'map-catalog:robot01:abc',
        'robot_id': 'robot01',
        'map_id': 'second',
        'action': 'DELETE',
    }))
    assert value.sent[-1]['accepted'] is False
    assert value.sent[-1]['detail'] == 'The active map cannot be changed'
    assert (tmp_path / 'second.yaml').is_file()
