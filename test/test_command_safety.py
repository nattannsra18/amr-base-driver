import time
from types import SimpleNamespace

from amr_base_driver.serial_bridge import SerialBridge


def state():
    sent = []
    obj = SimpleNamespace(last_cmd_monotonic=time.monotonic(), cmd_timeout=.25,
                          last_telemetry_monotonic=time.monotonic(), mcu_fault=0,
                          motors_enabled=True,
                          requested_linear=.05, requested_angular=0.0,
                          separation=.36094, radius=.032085, max_rpm=90,
                          command_sequence=0, serial_port=object(),
                          write_line=sent.append,
                          get_logger=lambda: SimpleNamespace(
                              warning=lambda _message: None,
                              error=lambda _message: None))
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


def test_clear_fault_forces_stop_before_clearing_mcu_fault():
    obj, sent = state()
    obj.mcu_fault = 2
    response = SimpleNamespace(success=False, message='')
    result = SerialBridge.clear_motor_fault(obj, None, response)
    assert sent == ['STOP', 'CLEAR']
    assert obj.requested_linear == 0.0
    assert obj.requested_angular == 0.0
    assert obj.last_cmd_monotonic == 0.0
    assert result.success
    assert 'mcu_fault=0' in result.message


def test_arm_forces_stop_and_requires_a_new_command():
    obj, sent = state()
    obj.motors_enabled = False
    response = SimpleNamespace(success=False, message='')
    request = SimpleNamespace(data=True)
    result = SerialBridge.set_motors_enabled(obj, request, response)
    assert sent == ['STOP']
    assert result.success
    assert obj.motors_enabled
    assert obj.requested_linear == 0.0
    assert obj.requested_angular == 0.0
    assert obj.last_cmd_monotonic == 0.0


def test_arm_rejects_stale_telemetry_and_stays_locked():
    obj, sent = state()
    obj.motors_enabled = False
    obj.last_telemetry_monotonic -= 1.0
    response = SimpleNamespace(success=True, message='')
    request = SimpleNamespace(data=True)
    result = SerialBridge.set_motors_enabled(obj, request, response)
    assert sent == ['STOP']
    assert not result.success
    assert not obj.motors_enabled
    assert 'telemetry stale' in result.message


def test_lock_is_always_allowed_and_forces_stop():
    obj, sent = state()
    obj.mcu_fault = 2
    response = SimpleNamespace(success=False, message='')
    request = SimpleNamespace(data=False)
    result = SerialBridge.set_motors_enabled(obj, request, response)
    assert sent == ['STOP']
    assert result.success
    assert not obj.motors_enabled
