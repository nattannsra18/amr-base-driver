import asyncio
from collections import OrderedDict
from datetime import datetime, timedelta, timezone
from queue import Queue
import threading
from types import SimpleNamespace

from amr_web_bridge.web_bridge_node import WebBridgeNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Goal:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1


def bridge():
    value = SimpleNamespace(
        emergency_stop_latched=threading.Event(),
        physical_estop_latched=threading.Event(),
        last_emergency_command_id=None,
        command_queue=Queue(),
        command_lock=threading.Lock(),
        pending_command_ids=set(),
        processed_command_ids=OrderedDict(),
        processed_command_limit=512,
        robot_id='robot01',
        profile_version='sim-v1',
        map_revision=7,
        pending_cancel_requests={},
        active_command={'command_id': 'old', 'task_id': 'task'},
        active_goal_handle=Goal(),
        emergency_velocity_publisher=Publisher(),
        sent=[],
    )
    async def send_json(_websocket, message): value.sent.append(message)
    value.send_json = send_json
    value.clear_command_queue = lambda: WebBridgeNode.clear_command_queue(value)
    value.publish_zero_velocity = lambda: WebBridgeNode.publish_zero_velocity(value)
    value.publish_emergency_zero = lambda: WebBridgeNode.publish_emergency_zero(value)
    value.motion_stop_latched = lambda: WebBridgeNode.motion_stop_latched(value)
    value.navigation_command_rejection = (
        lambda message: WebBridgeNode.navigation_command_rejection(value, message)
    )
    value.command_status_payload = (
        lambda command, lifecycle, detail: WebBridgeNode.command_status_payload(
            value, command, lifecycle, detail,
        )
    )
    value.clear_navigation_path = lambda *args, **kwargs: None
    value.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None,
        info=lambda _message: None,
    )
    return value


def run(value, command, command_id='command-1'):
    asyncio.run(WebBridgeNode.handle_emergency_command(
        value, object(), {'command': command, 'command_id': command_id}
    ))


def test_stop_latches_cancels_goal_clears_queue_and_publishes_zero():
    value = bridge()
    goal = value.active_goal_handle
    value.command_queue.put({'command_id': 'queued'})
    value.pending_command_ids.add('queued')
    run(value, 'emergency_stop')
    assert value.emergency_stop_latched.is_set()
    assert goal.cancel_count == 1
    assert value.command_queue.empty()
    assert value.pending_command_ids == set()
    assert value.active_command is None
    assert len(value.emergency_velocity_publisher.messages) == 1
    assert value.sent[-1]['type'] == 'emergency_ack'
    assert value.sent[-1]['accepted'] is True


def test_repeated_stop_is_idempotent_and_zero_timer_remains_bounded():
    value = bridge()
    run(value, 'emergency_stop', 'first')
    run(value, 'emergency_stop', 'second')
    WebBridgeNode.publish_emergency_zero(value)
    assert value.emergency_stop_latched.is_set()
    assert len(value.emergency_velocity_publisher.messages) == 3
    assert value.sent[-1]['command_id'] == 'second'


def test_navigation_is_rejected_while_latched():
    value = bridge()
    value.emergency_stop_latched.set()
    message = {
        'command_id': 'task:pickup:id', 'command': 'navigate_to_pose',
        'task_id': 'task', 'stage': 'pickup',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }
    asyncio.run(WebBridgeNode.queue_navigation_command(value, object(), message))
    assert value.command_queue.empty()
    assert value.sent[-1]['accepted'] is False
    assert 'latched' in value.sent[-1]['detail']


def test_navigation_is_rejected_while_physical_estop_is_latched():
    value = bridge()
    value.physical_estop_latched.set()
    message = {
        'command_id': 'task:pickup:id', 'command': 'navigate_to_pose',
        'task_id': 'task', 'stage': 'pickup',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }
    asyncio.run(WebBridgeNode.queue_navigation_command(value, object(), message))
    assert value.command_queue.empty()
    assert value.sent[-1]['accepted'] is False


def test_reset_clears_only_latch_and_never_replays_old_goal():
    value = bridge()
    run(value, 'emergency_stop')
    value.command_queue.put({'command_id': 'should-not-replay'})
    WebBridgeNode.clear_command_queue(value)
    run(value, 'emergency_stop_reset', 'reset-1')
    run(value, 'emergency_stop_reset', 'reset-2')
    assert not value.emergency_stop_latched.is_set()
    assert value.command_queue.empty()
    assert value.active_command is None
    assert value.sent[-1]['command'] == 'emergency_stop_reset'
    assert value.sent[-1]['accepted'] is True


def navigation_command(command_id='task:pickup:id', **overrides):
    message = {
        'command_id': command_id,
        'command': 'navigate_to_pose',
        'robot_id': 'robot01',
        'task_id': 'task',
        'stage': 'pickup',
        'target': {
            'frame_id': 'map',
            'x': 1.0,
            'y': 2.0,
            'yaw': 0.0,
        },
        'expires_at': (
            datetime.now(timezone.utc) + timedelta(seconds=30)
        ).isoformat(),
        'expected_profile_version': 'sim-v1',
    }
    message.update(overrides)
    return message


def test_navigation_rejects_expired_foreign_and_incompatible_commands():
    cases = [
        (
            {'expires_at': (
                datetime.now(timezone.utc) - timedelta(seconds=1)
            ).isoformat()},
            'expired',
        ),
        ({'robot_id': 'robot02'}, 'another robot'),
        ({'expected_profile_version': 'real-v2'}, 'profile version'),
        ({'expected_map_revision': 8}, 'Map revision'),
    ]

    for overrides, detail in cases:
        value = bridge()
        asyncio.run(WebBridgeNode.queue_navigation_command(
            value,
            object(),
            navigation_command(**overrides),
        ))
        assert value.command_queue.empty()
        assert value.sent[-1]['accepted'] is False
        assert detail in value.sent[-1]['detail']


def test_navigation_command_id_is_idempotent_after_leaving_queue():
    value = bridge()
    value.active_command = None
    command = navigation_command()

    asyncio.run(WebBridgeNode.queue_navigation_command(
        value, object(), command,
    ))
    value.command_queue.get_nowait()
    value.pending_command_ids.clear()

    asyncio.run(WebBridgeNode.queue_navigation_command(
        value, object(), command,
    ))

    assert value.command_queue.empty()
    assert value.sent[-1]['accepted'] is True
    assert 'already' in value.sent[-1]['detail']
