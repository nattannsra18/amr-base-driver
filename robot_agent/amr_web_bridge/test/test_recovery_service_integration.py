from concurrent.futures import Future
from types import SimpleNamespace

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
