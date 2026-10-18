from pathlib import Path


FIRMWARE = Path(__file__).parents[1] / (
    'firmware/esp32/robot_base_tb6612_mapping_v3_floor_start/'
    'robot_base_tb6612_mapping_v3_floor_start.ino'
)


def firmware_source():
    return FIRMWARE.read_text(encoding='utf-8')


def test_untrackable_wheel_target_is_stopped_instead_of_minimum_pwm():
    source = firmware_source()

    assert 'constexpr float MIN_TRACKABLE_RPM = 12.0f;' in source
    assert 'if (fabsf(target) < MIN_TRACKABLE_RPM)' in source
    assert source.index('if (fabsf(target) < MIN_TRACKABLE_RPM)') < source.index(
        'requested = max(requested, minimumRunningPwm);'
    )


def test_low_speed_fix_retains_firmware_motion_safety_boundaries():
    source = firmware_source()

    assert 'constexpr uint32_t COMMAND_WATCHDOG_MS = 300;' in source
    assert 'constexpr int MAX_PWM = 240;' in source
    assert 'if (faultCode != FAULT_NONE || !imuReady)' in source
    assert 'stopMotors();' in source
