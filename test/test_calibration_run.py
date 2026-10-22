from types import SimpleNamespace

from amr_base_driver.calibration_run import CalibrationRun
import pytest


def signs(mode, left, right):
    value = SimpleNamespace(mode=mode)
    return CalibrationRun.wheel_signs_match(value, left, right)


def test_controlled_motion_modes_require_expected_encoder_signs():
    assert signs('straight', 1.0, 1.1)
    assert signs('reverse', -1.0, -1.1)
    assert signs('arc_left', -1.2, -0.6)
    assert signs('arc_right', -0.6, -1.2)
    assert signs('arc_left', -1.2, 0.0)
    assert signs('arc_right', 0.0, -1.2)


def test_controlled_motion_rejects_wrong_or_missing_wheel_feedback():
    assert not signs('straight', 1.0, -1.0)
    assert not signs('reverse', -1.0, 0.0)
    assert not signs('arc_left', -0.4, -0.8)
    assert not signs('arc_right', -0.8, -0.4)


def test_independent_wheel_twist_commands_only_selected_wheel():
    radius = 0.032085
    separation = 0.36094
    rpm = 15.0
    wheel_speed = rpm * 2.0 * 3.141592653589793 * radius / 60.0

    left_linear, left_angular = CalibrationRun.independent_wheel_twist(
        'left', rpm, radius, separation)
    right_linear, right_angular = CalibrationRun.independent_wheel_twist(
        'right', rpm, radius, separation)

    assert left_linear == pytest.approx(wheel_speed / 2.0)
    assert right_linear == pytest.approx(wheel_speed / 2.0)
    assert left_angular == pytest.approx(-wheel_speed / separation)
    assert right_angular == pytest.approx(wheel_speed / separation)
    assert left_linear - left_angular * separation / 2.0 == pytest.approx(
        wheel_speed)
    assert left_linear + left_angular * separation / 2.0 == pytest.approx(0.0)
    assert right_linear - right_angular * separation / 2.0 == pytest.approx(0.0)
    assert right_linear + right_angular * separation / 2.0 == pytest.approx(
        wheel_speed)
