from amr_base_driver.motion_guard import MotionGuard


def test_grace_and_persistent_missing_feedback():
    guard = MotionGuard()
    guard.command(0.0, [15.0, 15.0])
    assert not guard.sample(0.84, [0.0, 0.0])
    assert not guard.sample(0.86, [15.0, 0.0])
    assert not guard.sample(1.30, [15.0, 0.0])
    assert not guard.sample(1.32, [15.0, 0.0])
    assert guard.pre_stall == 'RIGHT_NO_WHEEL_FEEDBACK'
    assert guard.state == 'PRE_STALL'
    assert guard.blocks_motion


def test_one_recovery_then_latches_if_feedback_is_still_missing():
    guard = MotionGuard()
    guard.command(0.0, [15.0, 15.0])
    guard.sample(0.86, [15.0, 0.0])
    guard.sample(1.32, [15.0, 0.0])
    assert guard.begin_recovery(1.33)
    assert guard.state == 'RECOVERING'
    assert not guard.blocks_motion
    guard.finish_recovery(True)
    guard.command(1.40, [15.0, 15.0])
    guard.sample(2.19, [15.0, 0.0])
    guard.sample(2.30, [15.0, 0.0])
    assert guard.sample(2.76, [15.0, 0.0]) == 'RIGHT_NO_WHEEL_FEEDBACK'
    assert guard.state == 'FAULTED'


def test_successful_feedback_rearms_a_future_pre_stall():
    guard = MotionGuard()
    guard.command(0.0, [15.0, 15.0])
    guard.sample(0.86, [15.0, 0.0])
    guard.sample(1.32, [15.0, 0.0])
    assert guard.begin_recovery(1.33)
    assert guard.finish_recovery(True)
    guard.command(1.40, [15.0, 15.0])
    guard.sample(2.30, [15.0, 15.0])
    guard.sample(4.31, [15.0, 15.0])
    assert guard.state == 'NORMAL'
    guard.sample(4.32, [15.0, 0.0])
    guard.sample(4.78, [15.0, 0.0])
    assert guard.state == 'PRE_STALL'


def test_failed_escape_latches_original_reason():
    guard = MotionGuard()
    guard.command(0.0, [15.0, 15.0])
    guard.sample(0.86, [0.0, 15.0])
    guard.sample(1.32, [0.0, 15.0])
    assert guard.begin_recovery(1.33)
    assert guard.finish_recovery(False)
    assert guard.fault == 'LEFT_NO_WHEEL_FEEDBACK'


def test_recovery_orchestrator_failure_can_latch_pre_stall_directly():
    guard = MotionGuard()
    guard.command(0.0, [15.0, 15.0])
    guard.sample(0.86, [15.0, 0.0])
    guard.sample(1.32, [15.0, 0.0])
    assert guard.finish_recovery(False)
    assert guard.fault == 'RIGHT_NO_WHEEL_FEEDBACK'


def test_reversal_and_low_targets_do_not_false_trip():
    guard = MotionGuard()
    guard.command(0.0, [15.0, 15.0])
    guard.sample(2.1, [0.0, 0.0])
    guard.command(2.3, [-15.0, -15.0])
    assert not guard.sample(3.7, [0.0, 0.0])
    guard.command(4.0, [3.0, 3.0])
    assert not guard.sample(20.0, [0.0, 0.0])


def test_pre_stall_is_raised_before_esp32_hard_latch_deadline():
    """The host must pause early enough for the automatic escape worker."""
    guard = MotionGuard()
    guard.command(0.0, [25.0, 25.0])
    guard.sample(0.86, [0.0, 25.0])
    guard.sample(1.32, [0.0, 25.0])

    assert guard.state == 'PRE_STALL'
    assert guard.pre_stall == 'LEFT_NO_WHEEL_FEEDBACK'
    assert 1.32 < 1.9
