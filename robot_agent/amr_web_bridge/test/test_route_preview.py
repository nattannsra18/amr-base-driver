import asyncio
import inspect
from queue import Queue
import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

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


def test_recovery_reuses_native_compute_path_action_client():
    source = inspect.getsource(WebBridgeNode.compute_recovery_path)
    init_source = inspect.getsource(WebBridgeNode.__init__)
    assert 'recovery_plan_client.send_goal_async' in source
    assert 'self.recovery_plan_client = self.preview_client' in init_source
    assert 'subprocess' not in source
    assert "['ros2'" not in source


def test_blocked_pose_recovery_does_not_launch_compute_path_cli():
    source = inspect.getsource(WebBridgeNode.run_blocked_pose_recovery)
    assert 'compute_recovery_path' in source
    assert 'navigation_recovery_runner.compute_path' not in source


class ImmediateFuture:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        callback(self)


class AcceptedRecoveryGoal:
    accepted = True

    def get_result_async(self):
        return ImmediateFuture(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(
                error_code=0,
                error_msg='',
                path=SimpleNamespace(poses=[object(), object()]),
            ),
        ))


class RecoveryPlanClient:
    def wait_for_server(self, timeout_sec):
        return timeout_sec == 1.0

    def send_goal_async(self, _goal):
        return ImmediateFuture(AcceptedRecoveryGoal())


class AcceptedSpinGoal:
    accepted = True

    def get_result_async(self):
        return ImmediateFuture(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(error_code=0),
        ))


class RecoverySpinClient:
    def wait_for_server(self, timeout_sec):
        return timeout_sec == 1.0

    def send_goal_async(self, goal):
        self.goal = goal
        return ImmediateFuture(AcceptedSpinGoal())


def test_native_recovery_spin_is_bounded_and_reports_success():
    client = RecoverySpinClient()
    node = SimpleNamespace(recovery_spin_client=client)
    turned, detail = WebBridgeNode.spin_for_recovery(node, 0.26)
    assert turned
    assert detail == 'Completed a bounded 0.26 rad turn'
    assert client.goal.target_yaw == 0.26
    assert client.goal.time_allowance.sec == 5


def test_native_recovery_plan_reports_connected_path():
    node = SimpleNamespace(recovery_plan_client=RecoveryPlanClient())
    node.build_recovery_plan_goal = lambda _target: object()

    planned, detail = WebBridgeNode.compute_recovery_path(
        node,
        {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    )

    assert planned is True
    assert detail == 'Nav2 computed a connected path with 2 poses'


def test_native_recovery_plan_has_bounded_timeout():
    class PendingFuture:
        def add_done_callback(self, _callback):
            return None

    class PendingRecoveryPlanClient(RecoveryPlanClient):
        def send_goal_async(self, _goal):
            return PendingFuture()

    node = SimpleNamespace(recovery_plan_client=PendingRecoveryPlanClient())
    node.build_recovery_plan_goal = lambda _target: object()

    planned, detail = WebBridgeNode.compute_recovery_path(
        node,
        {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
        timeout_seconds=0.01,
    )

    assert planned is False
    assert detail.startswith('Planner timeout:')


def near_goal_bridge(distance=0.28, *, localized=True, faulted=False):
    value = SimpleNamespace(
        near_goal_acceptance_distance_m=0.35,
        motion_stop_latched=lambda: False,
        localization_snapshot=lambda: {
            'health': 'LOCALIZED' if localized else 'LOST',
            'amcl_state': 'ACTIVE' if localized else 'INACTIVE',
            'tf_available': localized,
            'pose': {
                'frame_id': 'map',
                'x': distance,
                'y': 0.0,
                'yaw': 0.0,
            },
            'detail': 'ready' if localized else 'AMCL is inactive',
        },
        motor_fault_snapshot=lambda: {
            'mcu_fault': 2 if faulted else 0,
            'host_motion_fault': 'LEFT_ENCODER_STALL' if faulted else '',
            'host_motion_state': 'FAULT' if faulted else 'NORMAL',
            'host_motion_reason': '',
            'motors_enabled': not faulted,
        },
    )
    return value


def near_goal_command():
    return {
        'command_id': 'navigate:test-near-goal',
        'target': {'frame_id': 'map', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    }


def test_aborted_goal_is_accepted_inside_bounded_arrival_radius():
    accepted, detail = WebBridgeNode.accept_aborted_goal_when_near(
        near_goal_bridge(),
        near_goal_command(),
    )

    assert accepted is True
    assert '0.28 m from the goal' in detail


def test_aborted_goal_is_not_accepted_outside_arrival_radius():
    accepted, detail = WebBridgeNode.accept_aborted_goal_when_near(
        near_goal_bridge(distance=0.36),
        near_goal_command(),
    )

    assert accepted is False
    assert 'outside the 0.35 m acceptance radius' in detail


def test_aborted_goal_is_not_accepted_when_localization_is_lost():
    accepted, detail = WebBridgeNode.accept_aborted_goal_when_near(
        near_goal_bridge(localized=False),
        near_goal_command(),
    )

    assert accepted is False
    assert 'localization is not ready' in detail


def test_aborted_goal_is_not_accepted_when_motor_fault_is_present():
    accepted, detail = WebBridgeNode.accept_aborted_goal_when_near(
        near_goal_bridge(faulted=True),
        near_goal_command(),
    )

    assert accepted is False
    assert detail == 'ESP32 diagnostics are unavailable or motors are disabled'


def test_navigation_abort_near_goal_reports_success_without_escape_recovery():
    sent = []
    recovery_calls = []
    cleared = []
    value = SimpleNamespace(
        command_lock=threading.Lock(),
        recovery_cancelled_command_ids=set(),
        pending_cancel_requests={},
        accept_aborted_goal_when_near=lambda _command: (
            True,
            'Accepted arrival 0.28 m from the goal after Nav2 was blocked '
            'near the destination',
        ),
        maybe_start_blocked_pose_recovery=lambda command, detail: (
            recovery_calls.append((command, detail)) or True
        ),
        send_navigation_result=lambda command, status, detail: sent.append(
            (command, status, detail)
        ),
        clear_active_command=cleared.append,
        get_logger=lambda: SimpleNamespace(
            error=lambda _message: None,
            warning=lambda _message: None,
            info=lambda _message: None,
        ),
    )
    command = {
        'command_id': 'navigate:test-near-goal',
        'task_id': 'TASK-TEST',
        'target': {'frame_id': 'map', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    }
    future = ImmediateFuture(SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(error_msg='Nav2 goal was aborted'),
    ))

    WebBridgeNode.navigation_result_callback(value, future, command)

    assert sent == [(
        command,
        'succeeded',
        'Accepted arrival 0.28 m from the goal after Nav2 was blocked '
        'near the destination',
    )]
    assert recovery_calls == []
    assert cleared == ['navigate:test-near-goal']
