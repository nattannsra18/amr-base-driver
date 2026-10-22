from pathlib import Path
import struct
import threading
from types import SimpleNamespace

from amr_base_driver.serial_bridge import crc16_ccitt
from amr_base_driver.serial_bridge import SerialBridge
from amr_base_driver.serial_bridge import TELEMETRY_LENGTH
from amr_base_driver.serial_bridge import TELEMETRY_FORMAT
from amr_base_driver.serial_bridge import trim_straight_wheel_rpm
from amr_base_driver.serial_bridge import wheel_is_stalled
from amr_base_driver.serial_bridge import wrapped_int32_delta

import pytest
import yaml


def test_telemetry_wire_size():
    assert TELEMETRY_LENGTH == 44


def test_crc16_ccitt_false_reference_vector():
    assert crc16_ccitt(b'123456789') == 0x29B1


def telemetry_frame(mcu_ms=100, sequence=1):
    values = (
        0xA55A, 1, TELEMETRY_LENGTH, sequence, mcu_ms,
        10, 20, 100, 200,
        0, 0, 1000, 0, 0, 0, 2500, 7, 0, 0, 0,
    )
    frame = bytearray(struct.pack(TELEMETRY_FORMAT, *values))
    frame[-2:] = crc16_ccitt(frame[:-2]).to_bytes(2, 'little')
    return bytes(frame)


def telemetry_state():
    return SimpleNamespace(
        telemetry_lock=threading.Lock(),
        imu_angular_z_rps=0.0,
        mcu_flags=0,
        mcu_fault=0,
        encoder_left_count=0,
        encoder_right_count=0,
        measured_left_rpm=0.0,
        measured_right_rpm=0.0,
        last_telemetry_monotonic=0.0,
        telemetry_count=0,
        telemetry_revision=0,
        latest_telemetry=None,
        maximum_telemetry_gap=0.0,
        telemetry_gap_overrun_count=0,
        previous_telemetry_sequence=None,
        previous_telemetry_mcu_ms=None,
        maximum_mcu_telemetry_gap_ms=0,
        missed_telemetry_frames=0,
        mcu_restart_count=0,
        diagnostic_requested=False,
        parse_errors=0,
    )


def test_uart_reader_records_latest_frame_without_ros_publish():
    node = telemetry_state()

    SerialBridge.handle_telemetry_frame(node, telemetry_frame())

    assert node.telemetry_count == 1
    assert node.telemetry_revision == 1
    assert node.latest_telemetry['left_count'] == 10
    assert node.latest_telemetry['right_rpm'] == 2.0


def test_uart_reader_counts_interframe_gap_overruns(monkeypatch):
    node = telemetry_state()
    timestamps = iter((10.0, 10.35))
    monkeypatch.setattr('amr_base_driver.serial_bridge.time.monotonic',
                        lambda: next(timestamps))

    SerialBridge.handle_telemetry_frame(node, telemetry_frame(100, 1))
    SerialBridge.handle_telemetry_frame(node, telemetry_frame(450, 8))

    assert node.telemetry_gap_overrun_count == 1
    assert node.maximum_telemetry_gap == pytest.approx(0.35)
    assert node.maximum_mcu_telemetry_gap_ms == 350
    assert node.missed_telemetry_frames == 6


def test_uart_reader_counts_mcu_restart_without_overflowing_gap_metrics(
        monkeypatch):
    node = telemetry_state()
    timestamps = iter((10.0, 13.5))
    monkeypatch.setattr('amr_base_driver.serial_bridge.time.monotonic',
                        lambda: next(timestamps))

    SerialBridge.handle_telemetry_frame(node, telemetry_frame(5000, 100))
    SerialBridge.handle_telemetry_frame(node, telemetry_frame(50, 1))

    assert node.telemetry_gap_overrun_count == 1
    assert node.mcu_restart_count == 1
    assert node.maximum_mcu_telemetry_gap_ms == 0
    assert node.missed_telemetry_frames == 0


def test_wrapped_int32_delta_crosses_positive_boundary():
    assert wrapped_int32_delta(-0x80000000, 0x7FFFFFFF) == 1


def test_wrapped_int32_delta_crosses_negative_boundary():
    assert wrapped_int32_delta(0x7FFFFFFF, -0x80000000) == -1


def test_straight_trim_speeds_slower_wheel_without_changing_average():
    left, right = trim_straight_wheel_rpm(
        20.0, 20.0, 10.0, 20.0, 0.15, 3.0)
    assert (left, right) == (21.5, 18.5)


def test_straight_trim_is_bounded_and_does_not_change_turns():
    assert trim_straight_wheel_rpm(
        20.0, 20.0, 0.0, 30.0, 0.15, 3.0) == (23.0, 17.0)
    assert trim_straight_wheel_rpm(
        20.0, -20.0, 10.0, 20.0, 0.15, 3.0) == (20.0, -20.0)
    assert trim_straight_wheel_rpm(
        20.0, 20.0, 0.0, 0.0, 0.15, 3.0) == (20.0, 20.0)
    assert trim_straight_wheel_rpm(
        1.0, 1.0, 0.0, 30.0, 0.15, 3.0) == pytest.approx((1.9, 0.1))


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
        stall_guard_enabled=True,
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


def test_disabled_stall_guard_never_latches_motor_output():
    node = SimpleNamespace(
        stall_guard_enabled=False,
        stall_candidate_started=[10.0, 10.0],
        drive_blocked=False,
    )

    SerialBridge.update_stall_guard(node, 20.0, 30.0, 30.0)

    assert node.drive_blocked is False
    assert node.stall_candidate_started == [None, None]


def test_sensor_publish_rate_keeps_odometry_aligned_with_ekf():
    config = yaml.safe_load(
        (Path(__file__).parents[1] / 'config' / 'base_driver.yaml').read_text()
    )
    params = config['amr_base_serial_bridge']['ros__parameters']
    assert params['telemetry_publish_divisor'] == 1
    assert params['straight_trim_gain'] == 0.15
    assert params['straight_trim_max_rpm'] == 3.0
    assert params['stall_guard_enabled'] is False
    assert params['stall_guard_min_target_rpm'] == 20.0
    assert params['stall_guard_max_measured_rpm'] == 3.0
    assert params['stall_guard_timeout_seconds'] == 1.2
    assert params['stall_guard_recent_seconds'] == 30.0


def test_corrected_imu_uses_requested_gyro_variance():
    node = SimpleNamespace(imu_frame='imu_link')

    message = SerialBridge.make_imu_message(
        node, SimpleNamespace(), [0.1, 0.2, 0.3], [1.0, 2.0, 3.0], 4.0e-4)

    assert message.angular_velocity.z == 0.3
    assert message.linear_acceleration.y == 2.0
    assert message.angular_velocity_covariance[0] == 4.0e-4
    assert message.angular_velocity_covariance[4] == 4.0e-4
    assert message.angular_velocity_covariance[8] == 4.0e-4
