from types import SimpleNamespace

from amr_base_driver import nav2_activation_gate as gate_module
from amr_base_driver.nav2_activation_gate import (
    EXPECTED_FOOTPRINT,
    EXPECTED_RUNTIME_PARAMETERS,
    Nav2ActivationGate,
    validate_runtime_parameters,
)


def valid_runtime_values():
    values = {
        node: dict(parameters)
        for node, parameters in EXPECTED_RUNTIME_PARAMETERS.items()
    }
    for node in ('/local_costmap/local_costmap', '/global_costmap/global_costmap'):
        values[node]['footprint'] = repr(EXPECTED_FOOTPRINT)
    return values


def test_runtime_contract_accepts_physical_profile():
    assert validate_runtime_parameters(valid_runtime_values()) == []


def test_runtime_contract_rejects_composition_cross_wired_local_costmap():
    values = valid_runtime_values()
    local = values['/local_costmap/local_costmap']
    local.update({
        'global_frame': 'map',
        'rolling_window': False,
        'resolution': 0.03,
        'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
        'footprint': '[]',
    })

    errors = validate_runtime_parameters(values)

    assert any('global_frame' in error for error in errors)
    assert any('rolling_window' in error for error in errors)
    assert any('footprint' in error for error in errors)


def test_successful_startup_runs_contract_validation_before_exit():
    calls = []
    logger = SimpleNamespace(info=lambda message: calls.append(message))
    gate = SimpleNamespace(
        request_in_flight=True,
        navigation_started=False,
        timer=SimpleNamespace(cancel=lambda: calls.append('cancel')),
        validation_pending=[],
        get_logger=lambda: logger,
        request_next_parameters=lambda: calls.append('validate'),
    )
    future = SimpleNamespace(result=lambda: SimpleNamespace(success=True))

    Nav2ActivationGate.startup_complete(gate, future)

    assert gate.navigation_started is True
    assert gate.validation_pending == list(EXPECTED_RUNTIME_PARAMETERS)
    assert calls[-1] == 'validate'


def test_failed_contract_pauses_nav2(monkeypatch):
    requests = []
    shutdown_contexts = []

    class Client:
        def call_async(self, request):
            requests.append(request.command)
            return SimpleNamespace(
                add_done_callback=lambda callback: callback(
                    SimpleNamespace(
                        result=lambda: SimpleNamespace(success=True))))

    gate = SimpleNamespace(
        runtime_values={},
        count_publishers=lambda _topic: 1,
        client=Client(),
        context=object(),
        get_logger=lambda: SimpleNamespace(fatal=lambda _message: None),
    )
    monkeypatch.setattr(
        gate_module.rclpy,
        'shutdown',
        lambda *, context: shutdown_contexts.append(context),
    )

    Nav2ActivationGate.finish_validation(gate)

    assert requests == [gate_module.ManageLifecycleNodes.Request.PAUSE]
    assert shutdown_contexts == [gate.context]


def test_missing_filtered_odometry_publisher_pauses_nav2(monkeypatch):
    requests = []

    class Client:
        def call_async(self, request):
            requests.append(request.command)
            return SimpleNamespace(
                add_done_callback=lambda callback: callback(
                    SimpleNamespace(
                        result=lambda: SimpleNamespace(success=True))))

    gate = SimpleNamespace(
        runtime_values=valid_runtime_values(),
        count_publishers=lambda topic: 0 if topic == '/odometry/filtered' else 1,
        client=Client(),
        context=object(),
        get_logger=lambda: SimpleNamespace(fatal=lambda _message: None),
    )
    monkeypatch.setattr(gate_module.rclpy, 'shutdown', lambda *, context: None)

    Nav2ActivationGate.finish_validation(gate)

    assert requests == [gate_module.ManageLifecycleNodes.Request.PAUSE]
