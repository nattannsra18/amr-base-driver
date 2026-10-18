from pathlib import Path
from types import SimpleNamespace

from amr_base_driver.serial_bridge import crc16_ccitt
from amr_base_driver.serial_bridge import SerialBridge
from amr_base_driver.serial_bridge import suppress_untrackable_wheel_rpm
from amr_base_driver.serial_bridge import TELEMETRY_LENGTH
from amr_base_driver.serial_bridge import wheel_is_stalled
from amr_base_driver.serial_bridge import wrapped_int32_delta

import yaml


def test_telemetry_wire_size():
    assert TELEMETRY_LENGTH == 44


def test_crc16_ccitt_false_reference_vector():
    assert crc16_ccitt(b'123456789') == 0x29B1


def test_wrapped_int32_delta_crosses_positive_boundary():
    assert wrapped_int32_delta(-0x80000000, 0x7FFFFFFF) == 1


def test_wrapped_int32_delta_crosses_negative_boundary():
    assert wrapped_int32_delta(0x7FFFFFFF, -0x80000000) == -1


def test_untrackable_inside_wheel_is_stopped_for_tight_rolling_arc():
    assert suppress_untrackable_wheel_rpm(1.78, 33.94, 12.0) == (0.0, 33.94)
    assert suppress_untrackable_wheel_rpm(33.94, -1.78, 12.0) == (33.94, 0.0)


def test_trackable_wheel_targets_are_not_changed():
    assert suppress_untrackable_wheel_rpm(12.0, -20.0, 12.0) == (12.0, -20.0)


def test_stall_guard_requires_sustained_trackable_command_without_feedback():
    options = {
        'minimum_target_rpm': 20.0,
        'maximum_measured_rpm': 3.0,
        'timeout_seconds': 1.2,
    }
    assert wheel_is_stalled(30.0, 0.0, 1.2, **options)
    assert not wheel_is_stalled(19.9, 0.0, 2.0, **options)
    assert not wheel_is_stalled(30.0, 3.1, 2.0, **options)
    assert not wheel_is_stalled(30.0, 0.0, 1.19, **options)


def test_stall_guard_stops_both_outputs_after_sustained_encoder_stall():
    logs = []
    node = SimpleNamespace(
        stall_guard_min_target_rpm=20.0,
        stall_guard_max_measured_rpm=3.0,
        stall_guard_timeout_seconds=1.2,
        measured_left_rpm=0.0,
        measured_right_rpm=28.0,
        stall_candidate_started=[None, None],
        drive_blocked=False,
        blocked_left=False,
        blocked_right=False,
        blocked_target_rpm=(0.0, 0.0),
        last_drive_stall_monotonic=0.0,
        get_logger=lambda: SimpleNamespace(error=logs.append),
        diagnostic_tick=lambda: None,
    )

    SerialBridge.update_stall_guard(node, 10.0, 30.0, 30.0)
    SerialBridge.update_stall_guard(node, 11.21, 30.0, 30.0)

    assert node.drive_blocked is True
    assert node.blocked_left is True
    assert node.blocked_right is False
    assert node.blocked_target_rpm == (30.0, 30.0)
    assert 'stopped both motors' in logs[0]


def test_sensor_publish_rate_keeps_odometry_aligned_with_ekf():
    config = yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'base_driver.yaml').read_text()
    )
    params = config['amr_base_serial_bridge']['ros__parameters']
    assert params['telemetry_publish_divisor'] == 1
    assert params['minimum_trackable_rpm'] == 12.0
    assert params['stall_guard_min_target_rpm'] == 20.0
    assert params['stall_guard_max_measured_rpm'] == 3.0
    assert params['stall_guard_timeout_seconds'] == 1.2
    assert params['stall_guard_recent_seconds'] == 30.0
