from concurrent.futures import Future
from queue import Queue
import threading
import time
from types import SimpleNamespace

from amr_web_bridge.stall_escape import StallEscapeDecision
from amr_web_bridge.web_bridge_node import WebBridgeNode
from lifecycle_msgs.msg import State


def client(response, calls, name):
    def call(_request):
        calls.append(name)
        future = Future()
        future.set_result(response)
        return future

    return SimpleNamespace(wait_for_service=lambda **_kwargs: True, call_async=call)


def test_native_lifecycle_requires_active_numeric_state():
    calls = []
    inactive = SimpleNamespace(current_state=SimpleNamespace(id=2, label='inactive'))
    value = SimpleNamespace(recovery_lifecycle_clients=[
        ('/planner_server', client(inactive, calls, 'planner')),
        ('/controller_server', client(object(), calls, 'controller')),
    ])
    ok, detail = WebBridgeNode.recovery_lifecycle_healthy(value)
    assert not ok
    assert detail == '/planner_server: inactive'
    assert calls == ['planner']


def test_native_lifecycle_checks_all_required_nodes():
    calls = []
    active = SimpleNamespace(current_state=SimpleNamespace(id=3, label='active'))
    value = SimpleNamespace(recovery_lifecycle_clients=[
        (name, client(active, calls, name))
        for name in ('planner', 'controller', 'navigator')
    ])
    assert WebBridgeNode.recovery_lifecycle_healthy(value)[0]
    assert calls == ['planner', 'controller', 'navigator']


def test_lifecycle_wait_tolerates_one_inactive_sample():
    states = iter([
        (False, '/planner_server: inactive'),
        (True, 'Navigation lifecycle nodes are active'),
    ])
    value = SimpleNamespace(recovery_lifecycle_healthy=lambda: next(states))
    ok, detail = WebBridgeNode.wait_for_recovery_lifecycle(
        value,
        timeout_seconds=0.1,
        poll_seconds=0.0,
    )
    assert ok
    assert detail == 'Navigation lifecycle nodes are active'


def test_native_costmap_clear_calls_both_services_in_order():
    calls = []
    value = SimpleNamespace(recovery_costmap_clients=[
        (name, client(object(), calls, name)) for name in ('local', 'global')
    ])
    assert WebBridgeNode.clear_recovery_costmaps(value)[0]
    assert calls == ['local', 'global']


def test_costmap_clear_failure_stops_recovery():
    calls = []
    value = SimpleNamespace(recovery_costmap_clients=[
        ('local', client(None, calls, 'local')),
        ('global', client(object(), calls, 'global')),
    ])
    ok, detail = WebBridgeNode.clear_recovery_costmaps(value)
    assert not ok
    assert detail.startswith('local:')
    assert calls == ['local']


def test_guarded_reverse_uses_warm_service_client():
    calls = []
    response = SimpleNamespace(success=True, message='reverse complete')
    value = SimpleNamespace(
        guarded_reverse_client=client(response, calls, 'reverse'),
    )
    ok, detail = WebBridgeNode.guarded_reverse_for_recovery(value)
    assert ok
    assert detail == 'reverse complete'
    assert calls == ['reverse']


def test_guarded_arc_uses_direction_specific_warm_service_client():
    calls = []
    response = SimpleNamespace(success=True, message='arc complete')
    value = SimpleNamespace(
        guarded_arc_clients={
            1: client(response, calls, 'left'),
            -1: client(response, calls, 'right'),
        },
    )

    ok, detail = WebBridgeNode.guarded_arc_for_recovery(value, -0.22)

    assert ok
    assert detail == 'arc complete'
    assert calls == ['right']


def test_mapping_lifecycle_state_uses_generic_get_state_response():
    calls = []
    response = SimpleNamespace(
        current_state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE),
    )
    value = SimpleNamespace(
        mapping_lifecycle_clients={
            'planner_server': client(response, calls, 'planner'),
        },
    )

    assert WebBridgeNode.mapping_lifecycle_node_active(
        value,
        'planner_server',
    )
    assert calls == ['planner']


def test_mapping_lifecycle_command_uses_manage_nodes_response():
    calls = []
    value = SimpleNamespace(
        mapping_lifecycle_manager_clients={
            'lifecycle_manager_navigation': client(
                SimpleNamespace(success=True),
                calls,
                'navigation',
            ),
        },
    )

    WebBridgeNode.mapping_lifecycle_command(
        value,
        'lifecycle_manager_navigation',
        1,
    )
    assert calls == ['navigation']


def arc(value, side):
    return StallEscapeDecision(
        action='arc',
        value=value,
        clearance=0.8,
        detail=f'{side} clearance 0.80 m',
    )


def recovery_sequence_node(*, arc_results, extra_reverse=True):
    arcs = []
    reverses = []
    option_samples = iter([
        (arc(0.22, 'left'), arc(-0.22, 'right')),
        (arc(0.22, 'left'), arc(-0.22, 'right')),
    ])
    results = iter(arc_results)
    node = SimpleNamespace(
        stall_arc_decisions=lambda: next(option_samples),
        wait_for_recovery_motion_safe=lambda: (True, 'safe'),
        guarded_arc_for_recovery=lambda yaw: (
            arcs.append(yaw) or next(results)
        ),
        stall_escape_decision=lambda: StallEscapeDecision(
            action='back_up' if extra_reverse else 'none',
            value=0.10,
            clearance=0.8,
            detail='rear clearance 0.80 m',
        ),
        guarded_reverse_for_recovery=lambda: (
            reverses.append(0.10) or (True, 'reverse complete')
        ),
        wait_for_recovery_lifecycle=lambda: (True, 'active'),
        get_logger=lambda: SimpleNamespace(warning=lambda _message: None),
    )
    return node, arcs, reverses


def test_alternative_escape_stops_after_preferred_arc_succeeds():
    node, arcs, reverses = recovery_sequence_node(
        arc_results=[(True, 'left arc complete')],
    )

    ok, detail = WebBridgeNode.run_alternative_escape_maneuvers(node, 'cmd-1')

    assert ok
    assert arcs == [0.22]
    assert reverses == []
    assert 'left arc complete' in detail


def test_alternative_escape_reverses_again_then_tries_other_arc():
    node, arcs, reverses = recovery_sequence_node(
        arc_results=[
            (False, 'collision 703'),
            (False, 'collision 703'),
            (True, 'right arc complete'),
        ],
    )

    ok, detail = WebBridgeNode.run_alternative_escape_maneuvers(node, 'cmd-2')

    assert ok
    assert reverses == [0.10]
    assert arcs == [0.22, 0.22, -0.22]
    assert 'additional guarded reverse' in detail
    assert 'right arc complete' in detail


def test_alternative_escape_stops_when_safety_gate_changes():
    node, arcs, reverses = recovery_sequence_node(
        arc_results=[(False, 'collision 703')],
    )
    checks = iter([
        (True, 'safe'),
        (False, 'motor diagnostics are not normal'),
    ])
    node.wait_for_recovery_motion_safe = lambda: next(checks)

    ok, detail = WebBridgeNode.run_alternative_escape_maneuvers(node, 'cmd-3')

    assert not ok
    assert detail == 'motor diagnostics are not normal'
    assert arcs == [0.22]
    assert reverses == []


def test_alternative_escape_does_not_replan_after_arc_leaves_pre_stall():
    node, arcs, reverses = recovery_sequence_node(
        arc_results=[(False, 'right wheel feedback timed out')],
        extra_reverse=False,
    )
    checks = iter([
        (True, 'safe'),
        (False, 'motor diagnostics are not normal (PRE_STALL)'),
    ])
    node.wait_for_recovery_motion_safe = lambda: next(checks)

    ok, detail = WebBridgeNode.run_alternative_escape_maneuvers(
        node,
        'cmd-pre-stall',
        attempted_turn_values={-0.22},
    )

    assert not ok
    assert arcs == [0.22]
    assert reverses == []
    assert 'PRE_STALL' in detail


def test_recovery_motion_wait_allows_temporary_amcl_staleness_to_settle():
    states = iter([
        (False, 'localization is not ready (AMCL pose is stale while moving)'),
        (False, 'localization is not ready (AMCL pose is stale while moving)'),
        (True, 'recovery motion safety checks passed'),
    ])
    node = SimpleNamespace(recovery_motion_is_safe=lambda: next(states))

    ok, detail = WebBridgeNode.wait_for_recovery_motion_safe(
        node,
        timeout_seconds=0.1,
        poll_seconds=0.0,
    )

    assert ok
    assert detail == 'recovery motion safety checks passed'


def test_blocked_pose_recovery_turns_when_reverse_is_not_scan_safe():
    calls = []
    queue = Queue()
    node = SimpleNamespace(
        send_command_status=lambda *_args: calls.append('status'),
        spin_for_recovery=lambda yaw: (
            calls.append(('spin', yaw)) or (True, 'turn complete')
        ),
        guarded_reverse_for_recovery=lambda: (_ for _ in ()).throw(
            AssertionError('reverse must not run')
        ),
        wait_for_recovery_lifecycle=lambda: (True, 'active'),
        run_alternative_escape_maneuvers=lambda _command_id: (
            (_ for _ in ()).throw(AssertionError('extra maneuver must not run'))
        ),
        clear_recovery_costmaps=lambda: (True, 'cleared'),
        compute_recovery_path=lambda _target: (True, 'path available'),
        command_queue=queue,
        get_logger=lambda: SimpleNamespace(
            warning=lambda _message: None,
            error=lambda _message: None,
        ),
        blocked_start_recovery_lock=threading.Lock(),
        blocked_start_recovery_active=True,
        robot_operation_lock=threading.Lock(),
        robot_operation_active=True,
    )
    command = {
        'command_id': 'command-turn-only',
        'target': {'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }

    WebBridgeNode.run_blocked_pose_recovery(
        node,
        command,
        'Nav2 goal was aborted',
        'spin',
        0.26,
        'left clearance 0.50 m; rear clearance 0.38 m',
    )

    assert calls == ['status', ('spin', 0.26)]
    assert queue.get_nowait() == command
    assert node.blocked_start_recovery_active is False
    assert node.robot_operation_active is False


def test_blocked_pose_recovery_gate_accepts_scan_safe_turn(monkeypatch):
    calls = []
    node = SimpleNamespace(
        motion_stop_latched=lambda: False,
        blocked_start_recovery_window_seconds=60.0,
        stall_escape_decision=lambda: StallEscapeDecision(
            action='spin',
            value=0.26,
            clearance=0.50,
            detail='left clearance 0.50 m; rear clearance 0.38 m',
        ),
        motor_fault_snapshot=lambda: {
            'motors_enabled': True,
            'mcu_fault': 0,
            'host_motion_fault': '',
            'host_motion_state': 'NORMAL',
        },
        localization_snapshot=lambda: {
            'health': 'LOCALIZED',
            'amcl_state': 'ACTIVE',
            'tf_available': True,
        },
        blocked_start_recovery_lock=threading.Lock(),
        blocked_start_recovery_active=False,
        blocked_start_recovery_attempted_ids=set(),
        robot_operation_lock=threading.Lock(),
        robot_operation_active=False,
        clear_active_command=lambda command_id: calls.append(('clear', command_id)),
        run_blocked_pose_recovery=lambda *args: calls.append(('run', args)),
        get_logger=lambda: SimpleNamespace(warning=lambda _message: None),
    )

    class ImmediateThread:
        def __init__(self, *, target, args, **_kwargs):
            self.target = target
            self.args = args

        def start(self):
            self.target(*self.args)

    monkeypatch.setattr(threading, 'Thread', ImmediateThread)
    command = {
        'command_id': 'command-turn-gate',
        '_navigation_started_monotonic': time.monotonic(),
    }

    started = WebBridgeNode.maybe_start_blocked_pose_recovery(
        node,
        command,
        'Nav2 goal was aborted',
    )

    assert started is True
    assert calls[0] == ('clear', 'command-turn-gate')
    assert calls[1][0] == 'run'
    assert calls[1][1][2:5] == (
        'spin',
        0.26,
        'left clearance 0.50 m; rear clearance 0.38 m',
    )


def test_recovery_motion_wait_does_not_delay_a_motor_fault():
    calls = []
    node = SimpleNamespace(
        recovery_motion_is_safe=lambda: (
            calls.append(True) or (False, 'motor diagnostics are not normal')
        ),
    )

    ok, detail = WebBridgeNode.wait_for_recovery_motion_safe(node)

    assert not ok
    assert detail == 'motor diagnostics are not normal'
    assert calls == [True]
