from types import SimpleNamespace

from amr_base_driver.calibration_run import CalibrationRun


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
