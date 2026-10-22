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
        mapping_active=lambda: False,
        lifecycle_interlock=SimpleNamespace(
            release=lambda: calls.append('release')),
        retry_delay_seconds=4.0,
        next_retry_monotonic=99.0,
        timer=SimpleNamespace(cancel=lambda: calls.append('cancel')),
        validation_pending=[],
        get_logger=lambda: logger,
        begin_validation=lambda message: calls.extend([message, 'validate']),
    )
    future = SimpleNamespace(result=lambda: SimpleNamespace(success=True))

    Nav2ActivationGate.startup_complete(gate, future)

    assert gate.navigation_started is False
    assert gate.retry_delay_seconds == 0.5
    assert gate.next_retry_monotonic == 0.0
    assert calls[-1] == 'validate'


def test_runtime_parameter_clients_are_created_only_during_validation(monkeypatch):
    created = []
    monkeypatch.setattr(
        gate_module,
        'AsyncParameterClient',
        lambda _node, name: created.append(name) or SimpleNamespace(),
    )
    gate = SimpleNamespace(
        retry_delay_seconds=4.0,
        next_retry_monotonic=99.0,
        navigation_started=False,
        timer=SimpleNamespace(cancel=lambda: None),
        parameter_clients={},
        validation_pending=[],
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
        request_next_parameters=lambda: None,
    )

    Nav2ActivationGate.begin_validation(gate, 'ready')

    assert created == list(EXPECTED_RUNTIME_PARAMETERS)
    assert set(gate.parameter_clients) == set(EXPECTED_RUNTIME_PARAMETERS)


def test_mapping_marker_blocks_navigation_activation():
    gate = SimpleNamespace(
        mapping_active=lambda: True,
        navigation_started=False,
        request_in_flight=False,
        buffer=SimpleNamespace(
            can_transform=lambda *_args: (_ for _ in ()).throw(
                AssertionError('TF must not be checked during mapping')
            )
        ),
    )

    Nav2ActivationGate.check_localization(gate)


def test_lifecycle_interlock_blocks_navigation_activation():
    calls = []
    gate = SimpleNamespace(
        mapping_active=lambda: False,
        navigation_started=False,
        request_in_flight=False,
        next_retry_monotonic=0.0,
        global_frame='map',
        base_frame='base_footprint',
        localization_is_fresh=lambda: True,
        client=SimpleNamespace(service_is_ready=lambda: True),
        planner_state_client=SimpleNamespace(service_is_ready=lambda: True),
        lifecycle_interlock=SimpleNamespace(
            try_acquire=lambda: calls.append('busy') or False,
        ),
    )

    Nav2ActivationGate.check_localization(gate)

    assert calls == ['busy']


def test_localization_rejects_tf_without_current_session_amcl_pose():
    gate = SimpleNamespace(
        latest_amcl_pose_stamp=None,
        buffer=SimpleNamespace(
            lookup_transform=lambda *_args: (_ for _ in ()).throw(
                AssertionError('TF must not be enough without an AMCL pose'))),
    )

    assert Nav2ActivationGate.localization_is_fresh(gate) is False


def test_tf_listener_is_started_only_after_current_session_amcl_pose(monkeypatch):
    created = []
    monkeypatch.setattr(
        gate_module,
        'TransformListener',
        lambda buffer, node: created.append((buffer, node)) or object(),
    )
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=gate_module.Time(seconds=100).to_msg()))
    gate = SimpleNamespace(listener=None, buffer=object())

    Nav2ActivationGate.amcl_pose_received(gate, message)

    assert gate.latest_amcl_pose_stamp.nanoseconds == 100_000_000_000
    assert created == [(gate.buffer, gate)]
    assert gate.listener is not None


def test_localization_rejects_stale_amcl_pose():
    now = gate_module.Time.from_msg(gate_module.Time(seconds=100).to_msg())
    gate = SimpleNamespace(
        latest_amcl_pose_stamp=gate_module.Time.from_msg(
            gate_module.Time(seconds=97).to_msg()),
        max_amcl_pose_age_seconds=2.0,
        timestamp_age_seconds=Nav2ActivationGate.timestamp_age_seconds,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        buffer=SimpleNamespace(
            lookup_transform=lambda *_args: (_ for _ in ()).throw(
                AssertionError('TF must not be checked after stale pose'))),
    )

    assert Nav2ActivationGate.localization_is_fresh(gate) is False


def test_localization_rejects_stale_transform():
    now = gate_module.Time.from_msg(gate_module.Time(seconds=100).to_msg())
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=gate_module.Time(seconds=98).to_msg()))
    gate = SimpleNamespace(
        latest_amcl_pose_stamp=gate_module.Time.from_msg(
            gate_module.Time(seconds=99).to_msg()),
        max_amcl_pose_age_seconds=2.0,
        max_transform_age_seconds=1.0,
        max_transform_future_seconds=1.25,
        global_frame='map',
        base_frame='base_footprint',
        timestamp_age_seconds=Nav2ActivationGate.timestamp_age_seconds,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        buffer=SimpleNamespace(lookup_transform=lambda *_args: transform),
    )

    assert Nav2ActivationGate.localization_is_fresh(gate) is False


def test_localization_accepts_fresh_pose_and_transform():
    now = gate_module.Time.from_msg(gate_module.Time(seconds=100).to_msg())
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=gate_module.Time(
            seconds=99, nanoseconds=500_000_000).to_msg()))
    gate = SimpleNamespace(
        latest_amcl_pose_stamp=gate_module.Time.from_msg(
            gate_module.Time(seconds=99).to_msg()),
        max_amcl_pose_age_seconds=2.0,
        max_transform_age_seconds=1.0,
        max_transform_future_seconds=1.25,
        global_frame='map',
        base_frame='base_footprint',
        timestamp_age_seconds=Nav2ActivationGate.timestamp_age_seconds,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        buffer=SimpleNamespace(lookup_transform=lambda *_args: transform),
    )

    assert Nav2ActivationGate.localization_is_fresh(gate) is True


def test_localization_accepts_amcl_future_dated_transform():
    now = gate_module.Time.from_msg(gate_module.Time(seconds=100).to_msg())
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=gate_module.Time(
            seconds=101).to_msg()))
    gate = SimpleNamespace(
        latest_amcl_pose_stamp=gate_module.Time.from_msg(
            gate_module.Time(seconds=99).to_msg()),
        max_amcl_pose_age_seconds=2.0,
        max_transform_age_seconds=1.0,
        max_transform_future_seconds=1.25,
        global_frame='map',
        base_frame='base_footprint',
        timestamp_age_seconds=Nav2ActivationGate.timestamp_age_seconds,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        buffer=SimpleNamespace(lookup_transform=lambda *_args: transform),
    )

    assert Nav2ActivationGate.localization_is_fresh(gate) is True


def test_localization_rejects_transform_too_far_in_future():
    now = gate_module.Time.from_msg(gate_module.Time(seconds=100).to_msg())
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=gate_module.Time(
            seconds=102).to_msg()))
    gate = SimpleNamespace(
        latest_amcl_pose_stamp=gate_module.Time.from_msg(
            gate_module.Time(seconds=99).to_msg()),
        max_amcl_pose_age_seconds=2.0,
        max_transform_age_seconds=1.0,
        max_transform_future_seconds=1.25,
        global_frame='map',
        base_frame='base_footprint',
        timestamp_age_seconds=Nav2ActivationGate.timestamp_age_seconds,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        buffer=SimpleNamespace(lookup_transform=lambda *_args: transform),
    )

    assert Nav2ActivationGate.localization_is_fresh(gate) is False


def test_already_active_nav2_validates_without_startup_request():
    calls = []
    gate = SimpleNamespace(
        request_in_flight=True,
        mapping_active=lambda: False,
        lifecycle_interlock=SimpleNamespace(release=lambda: calls.append('release')),
        begin_validation=lambda message: calls.extend([message, 'validate']),
        client=SimpleNamespace(
            call_async=lambda _request: (_ for _ in ()).throw(
                AssertionError('STARTUP must not be sent to active Nav2'))),
    )
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        current_state=SimpleNamespace(id=3)))

    Nav2ActivationGate.planner_state_complete(gate, future)

    assert gate.request_in_flight is False
    assert calls[-1] == 'validate'


def test_mapping_request_during_startup_defers_without_validation():
    calls = []
    gate = SimpleNamespace(
        request_in_flight=True,
        navigation_started=False,
        mapping_active=lambda: True,
        lifecycle_interlock=SimpleNamespace(
            release=lambda: calls.append('release')),
        get_logger=lambda: SimpleNamespace(info=lambda message: calls.append(message)),
        begin_validation=lambda message: calls.extend([message, 'validate']),
    )
    future = SimpleNamespace(result=lambda: SimpleNamespace(success=True))

    Nav2ActivationGate.startup_complete(gate, future)

    assert gate.navigation_started is False
    assert calls[-1] == 'release'
    assert 'validate' not in calls


def test_rejected_startup_resets_partial_lifecycle_once():
    requests = []
    callbacks = []
    calls = []

    class Client:
        def call_async(self, request):
            requests.append(request.command)
            return SimpleNamespace(add_done_callback=callbacks.append)

    gate = SimpleNamespace(
        request_in_flight=True,
        lifecycle_repair_attempted=False,
        mapping_active=lambda: False,
        client=Client(),
        request_lifecycle_reset=lambda: (
            Nav2ActivationGate.request_lifecycle_reset(gate)),
        lifecycle_reset_complete=lambda future: (
            Nav2ActivationGate.lifecycle_reset_complete(gate, future)),
        stop_after_failed_repair=lambda message: calls.append(message),
        get_logger=lambda: SimpleNamespace(warning=lambda message: calls.append(message)),
    )

    Nav2ActivationGate.startup_complete(
        gate,
        SimpleNamespace(result=lambda: SimpleNamespace(success=False)),
    )

    assert gate.lifecycle_repair_attempted is True
    assert gate.request_in_flight is True
    assert requests == [gate_module.ManageLifecycleNodes.Request.RESET]
    assert len(callbacks) == 1


def test_successful_lifecycle_reset_retries_with_backoff():
    calls = []
    gate = SimpleNamespace(
        request_in_flight=True,
        defer_retry=lambda: calls.append('retry'),
        lifecycle_interlock=SimpleNamespace(
            release=lambda: calls.append('release')),
        stop_after_failed_repair=lambda message: calls.append(message),
        get_logger=lambda: SimpleNamespace(info=lambda message: calls.append(message)),
    )

    Nav2ActivationGate.lifecycle_reset_complete(
        gate,
        SimpleNamespace(result=lambda: SimpleNamespace(success=True)),
    )

    assert gate.request_in_flight is False
    assert calls[-2:] == ['retry', 'release']


def test_second_rejected_startup_stops_fail_closed():
    calls = []
    gate = SimpleNamespace(
        request_in_flight=True,
        lifecycle_repair_attempted=True,
        mapping_active=lambda: False,
        stop_after_failed_repair=lambda message: calls.append(message),
    )

    Nav2ActivationGate.startup_complete(
        gate,
        SimpleNamespace(result=lambda: SimpleNamespace(success=False)),
    )

    assert calls == [
        'Nav2 lifecycle startup was rejected after one reset; activation is '
        'stopping fail-closed'
    ]


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
        lifecycle_interlock=SimpleNamespace(release=lambda: None),
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
        lifecycle_interlock=SimpleNamespace(release=lambda: None),
        get_logger=lambda: SimpleNamespace(fatal=lambda _message: None),
    )
    monkeypatch.setattr(gate_module.rclpy, 'shutdown', lambda *, context: None)

    Nav2ActivationGate.finish_validation(gate)

    assert requests == [gate_module.ManageLifecycleNodes.Request.PAUSE]
