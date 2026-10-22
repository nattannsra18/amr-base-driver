from types import SimpleNamespace

from amr_base_driver import localization_readiness_gate as gate_module
from amr_base_driver.localization_readiness_gate import (
    LocalizationReadinessGate,
)


def test_mapping_marker_blocks_navigation_process_launch():
    gate = SimpleNamespace(
        mapping_active=lambda: True,
        ready=False,
        state_requests=0,
        startup_pending=False,
        state_clients={
            'amcl': SimpleNamespace(
                service_is_ready=lambda: (_ for _ in ()).throw(
                    AssertionError(
                        'lifecycle readiness must not run during mapping'
                    )
                )
            )
        },
    )

    LocalizationReadinessGate.check(gate)


def test_main_exits_zero_only_after_localization_is_ready(monkeypatch):
    node = SimpleNamespace(
        ready=True,
        destroy_node=lambda: None,
    )
    monkeypatch.setattr(gate_module.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(gate_module.rclpy, 'spin', lambda _node: None)
    monkeypatch.setattr(gate_module.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(gate_module, 'LocalizationReadinessGate', lambda: node)

    assert gate_module.main() == 0


def test_main_exits_nonzero_when_stopped_before_ready(monkeypatch):
    node = SimpleNamespace(
        ready=False,
        destroy_node=lambda: None,
    )
    monkeypatch.setattr(gate_module.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(
        gate_module.rclpy,
        'spin',
        lambda _node: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    monkeypatch.setattr(gate_module.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(gate_module, 'LocalizationReadinessGate', lambda: node)

    assert gate_module.main() == 1
