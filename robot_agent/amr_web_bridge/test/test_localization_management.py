import asyncio
from queue import Queue
import threading
import time
from types import SimpleNamespace

from amr_web_bridge.web_bridge_node import WebBridgeNode


def bridge(*, pose_age=0.2, moving=False, uncertainty=0.1, mapping=False):
    now = time.monotonic()
    value = SimpleNamespace(
        robot_id='robot01',
        active_map_id='warehouse_map',
        map_revision=4,
        node_started_monotonic=now - 20.0,
        localization_lock=threading.Lock(),
        velocity_lock=threading.Lock(),
        command_lock=threading.Lock(),
        mapping_velocity_lock=threading.Lock(),
        latest_localization_pose={
            'frame_id': 'map', 'x': 1.0, 'y': -2.0, 'yaw': 0.4,
            'position_uncertainty': uncertainty,
            'yaw_uncertainty': uncertainty,
        },
        last_localization_monotonic=now - pose_age,
        amcl_state='ACTIVE',
        localization_recovery_count=0,
        localization_recovery_active=False,
        localization_recovery_started_monotonic=None,
        localization_convergence_samples=0,
        localization_scan_active=False,
        localization_scan_started_monotonic=None,
        localization_scan_angular_speed=0.26,
        localization_scan_timeout_seconds=28.0,
        latest_velocity={
            'linear_velocity': 0.1 if moving else 0.0,
            'angular_velocity': 0.0,
        },
        mapping_runtime=SimpleNamespace(
            snapshot=lambda _revision: {
                'phase': 'MAPPING' if mapping else 'IDLE'
            }
        ),
        lookup_planar_pose=lambda _parent, _child: (1.1, -2.1, 0.45),
        localization_command_queue=Queue(),
        active_localization_command=None,
        active_command=None,
        localization_lifecycle_client=SimpleNamespace(
            service_is_ready=lambda: False,
        ),
        emergency_stop_latched=threading.Event(),
        physical_estop_latched=threading.Event(),
        mapping_teleop_deadman_seconds=0.35,
        localization_velocity_deadline=0.0,
        localization_velocity_active=False,
        sent=[],
        publish_zero_velocity=lambda: None,
    )
    value.localization_snapshot = lambda: WebBridgeNode.localization_snapshot(value)
    value.motion_stop_latched = (
        lambda: WebBridgeNode.motion_stop_latched(value)
    )
    value.send_from_ros = value.sent.append
    value.finish_localization_command = (
        lambda command, accepted, detail:
        WebBridgeNode.finish_localization_command(
            value, command, accepted, detail
        )
    )
    value.publish_initial_pose_command = (
        lambda command: WebBridgeNode.publish_initial_pose_command(value, command)
    )
    value.localization_resume_callback = (
        lambda future, command:
        WebBridgeNode.localization_resume_callback(value, future, command)
    )
    return value


def test_stationary_pose_remains_localized_when_amcl_message_is_old():
    snapshot = WebBridgeNode.localization_snapshot(
        bridge(pose_age=30.0, moving=False)
    )
    assert snapshot['health'] == 'LOCALIZED'
    assert snapshot['reason'] == 'READY'
    assert snapshot['tf_available'] is True


def test_moving_stale_pose_is_lost_and_high_uncertainty_is_degraded():
    stale = WebBridgeNode.localization_snapshot(
        bridge(pose_age=4.0, moving=True)
    )
    assert stale['health'] == 'LOST'
    assert stale['reason'] == 'POSE_STALE'
    uncertain = WebBridgeNode.localization_snapshot(
        bridge(uncertainty=1.2)
    )
    assert uncertain['health'] == 'DEGRADED'
    assert uncertain['reason'] == 'HIGH_UNCERTAINTY'


def test_mapping_pauses_localization_health():
    snapshot = WebBridgeNode.localization_snapshot(bridge(mapping=True))
    assert snapshot['health'] == 'UNKNOWN'
    assert snapshot['reason'] == 'MAPPING_ACTIVE'


def test_initial_pose_command_is_published_and_acknowledged():
    value = bridge()
    published = []
    value.publish_initial_pose = lambda *args, **kwargs: published.append(
        (args, kwargs)
    )
    command = {
        'action': 'SET_INITIAL_POSE',
        'command_id': 'localization-set_initial_pose:robot01:test',
        'pose': {'frame_id': 'map', 'x': 2.0, 'y': 3.0, 'yaw': 0.2},
        'position_uncertainty': 0.4,
        'yaw_uncertainty': 0.3,
    }
    value.localization_command_queue.put(command)
    WebBridgeNode.process_localization_command_queue(value)
    assert published == [((2.0, 3.0, 0.2), {
        'position_uncertainty': 0.4,
        'yaw_uncertainty': 0.3,
    })]
    assert value.sent[-1]['command_action'] == 'SET_INITIAL_POSE'
    assert value.sent[-1]['accepted'] is True
    assert value.localization_recovery_count == 1
    assert value.localization_recovery_active is True


def test_amcl_state_callback_accepts_active_lifecycle_id():
    value = bridge()
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        current_state=SimpleNamespace(id=3, label='ACTIVE '),
    ))
    value.amcl_state_future = future
    value.amcl_state_request_started_monotonic = time.monotonic()
    value.amcl_state_last_success_monotonic = None
    WebBridgeNode.amcl_state_callback(value, future)
    assert value.amcl_state == 'ACTIVE'
    assert value.amcl_state_future is None
    assert value.amcl_state_last_success_monotonic is not None


def test_amcl_state_poll_retries_a_stuck_lifecycle_request():
    value = bridge()
    stuck = SimpleNamespace(done=lambda: False, cancel=lambda: None)
    replacement = SimpleNamespace(add_done_callback=lambda callback: None)
    value.amcl_state_future = stuck
    value.amcl_state_request_started_monotonic = time.monotonic() - 4.0
    value.amcl_state_last_success_monotonic = None
    value.amcl_state_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: replacement,
    )
    value.amcl_state_callback = (
        lambda future: WebBridgeNode.amcl_state_callback(value, future)
    )

    WebBridgeNode.poll_amcl_state(value)

    assert value.amcl_state_future is replacement
    assert value.amcl_state_request_started_monotonic is not None


def test_transient_amcl_service_loss_keeps_recent_active_state():
    value = bridge()
    value.amcl_state_future = None
    value.amcl_state_request_started_monotonic = None
    value.amcl_state_last_success_monotonic = time.monotonic()
    value.amcl_state_client = SimpleNamespace(service_is_ready=lambda: False)

    WebBridgeNode.poll_amcl_state(value)

    assert value.amcl_state == 'ACTIVE'


def test_inactive_amcl_is_resumed_before_initial_pose_is_published():
    value = bridge()
    value.amcl_state = 'INACTIVE'
    published = []
    callbacks = []
    value.publish_initial_pose = lambda *args, **kwargs: published.append(
        (args, kwargs)
    )
    future = SimpleNamespace(add_done_callback=callbacks.append)
    value.localization_lifecycle_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: future,
    )
    command = {
        'action': 'SET_INITIAL_POSE',
        'command_id': 'localization-set_initial_pose:test',
        'pose': {'frame_id': 'map', 'x': 2.0, 'y': 3.0, 'yaw': 0.2},
        'position_uncertainty': 0.4,
        'yaw_uncertainty': 0.3,
    }
    value.localization_command_queue.put(command)
    WebBridgeNode.process_localization_command_queue(value)
    assert published == []
    assert len(callbacks) == 1

    callbacks[0](SimpleNamespace(result=lambda: SimpleNamespace(success=True)))
    assert published
    assert value.sent[-1]['accepted'] is True
    assert value.localization_recovery_active is True


def test_recovery_teleop_is_limited_and_deadman_stops_motion():
    value = bridge()
    value.localization_recovery_active = True
    published = []
    value.emergency_velocity_publisher = SimpleNamespace(
        publish=published.append
    )
    asyncio.run(WebBridgeNode.handle_localization_teleop(value, None, {
        'type': 'localization_teleop',
        'robot_id': 'robot01',
        'linear_x': 0.08,
        'angular_z': 0.2,
    }))
    assert published[-1].linear.x == 0.08
    assert published[-1].angular.z == 0.2
    assert value.localization_velocity_active is True
    value.localization_velocity_deadline = time.monotonic() - 0.01
    stopped = []
    value.publish_zero_velocity = lambda: stopped.append(True)
    value.mapping_velocity_active = False
    WebBridgeNode.enforce_mapping_deadman(value)
    assert stopped == [True]
    assert value.localization_velocity_active is False


def test_recovery_teleop_is_rejected_outside_global_search():
    value = bridge()
    published = []
    value.emergency_velocity_publisher = SimpleNamespace(
        publish=published.append
    )
    value.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    asyncio.run(WebBridgeNode.handle_localization_teleop(value, None, {
        'type': 'localization_teleop',
        'robot_id': 'robot01',
        'linear_x': 0.08,
        'angular_z': 0.0,
    }))
    assert published == []


def test_automatic_scan_rotates_and_stops_on_timeout():
    value = bridge()
    value.localization_recovery_active = True
    published = []
    stopped = []
    value.emergency_velocity_publisher = SimpleNamespace(
        publish=published.append
    )
    value.publish_zero_velocity = lambda: stopped.append(True)
    value.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    asyncio.run(WebBridgeNode.handle_localization_scan(value, None, {
        'type': 'localization_scan',
        'robot_id': 'robot01',
        'action': 'START',
    }))
    WebBridgeNode.run_localization_scan(value)
    assert published[-1].angular.z == 0.26
    value.localization_scan_started_monotonic = time.monotonic() - 29.0
    WebBridgeNode.run_localization_scan(value)
    assert stopped == [True]
    assert value.localization_scan_active is False
