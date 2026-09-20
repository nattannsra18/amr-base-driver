import time
from types import SimpleNamespace
from amr_base_driver.motion_guard import MotionGuard
from amr_base_driver.serial_bridge import SerialBridge


def state():
    sent = []
    obj = SimpleNamespace(last_cmd_monotonic=time.monotonic(), cmd_timeout=.25,
                          last_telemetry_monotonic=time.monotonic(), mcu_fault=0,
                          motion_guard=MotionGuard(), motors_enabled=True,
                          requested_linear=.05, requested_angular=0.0,
                          separation=.36094, radius=.032085, max_rpm=90,
                          command_sequence=0, write_line=sent.append)
    return obj, sent


def test_stale_telemetry_stops_even_when_keyboard_is_fresh():
    obj, sent = state()
    obj.last_telemetry_monotonic -= 1.0
    SerialBridge.command_tick(obj)
    assert sent == ['STOP']


def test_fault_never_cleared_or_driven_by_fresh_keyboard():
    obj, sent = state()
    obj.mcu_fault = 4
    SerialBridge.command_tick(obj)
    assert sent == ['STOP']


def test_normal_command_and_disabled_motor():
    obj, sent = state()
    SerialBridge.command_tick(obj)
    fields = sent[0].split(',')
    assert fields[:2] == ['CMD', '0']
    assert 14.8 < float(fields[2]) < 15.0
    assert fields[2] == fields[3]
    obj.motors_enabled = False
    SerialBridge.command_tick(obj)
    assert sent[-1] == 'CMD,1,0.000,0.000'


def test_clear_fault_forces_stop_before_clear_and_resets_host_guard():
    obj, sent = state()
    obj.mcu_fault = 2
    obj.motion_guard.fault = 'LEFT_NO_WHEEL_FEEDBACK'
    response = SimpleNamespace(success=False, message='')
    result = SerialBridge.clear_motor_fault(obj, None, response)
    assert sent == ['STOP', 'CLEAR']
    assert obj.requested_linear == 0.0
    assert obj.requested_angular == 0.0
    assert obj.last_cmd_monotonic == 0.0
    assert not obj.motion_guard.fault
    assert result.success
    assert 'mcu_fault=0' in result.message
