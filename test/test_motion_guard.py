from amr_base_driver.motion_guard import MotionGuard


def test_grace_and_persistent_missing_feedback():
    guard = MotionGuard()
    guard.command(0.0, [15.0, 15.0])
    assert not guard.sample(1.9, [0.0, 0.0])
    assert not guard.sample(2.1, [15.0, 0.0])
    assert guard.sample(3.7, [15.0, 0.0]) == 'RIGHT_NO_WHEEL_FEEDBACK'
    guard.command(4.0, [0.0, 0.0])
    assert guard.fault == 'RIGHT_NO_WHEEL_FEEDBACK'


def test_reversal_and_low_targets_do_not_false_trip():
    guard = MotionGuard()
    guard.command(0.0, [15.0, 15.0])
    guard.sample(2.1, [0.0, 0.0])
    guard.command(2.3, [-15.0, -15.0])
    assert not guard.sample(3.7, [0.0, 0.0])
    guard.command(4.0, [3.0, 3.0])
    assert not guard.sample(20.0, [0.0, 0.0])
