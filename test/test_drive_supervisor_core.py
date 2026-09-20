import math

from amr_base_driver.drive_supervisor_core import DriveSupervisorCore


def tick(core, now, linear, angular=0.0, **kwargs):
    return core.update(now, 0.05, linear, angular, **kwargs)


def test_zero_and_stale_commands_stop_immediately():
    core = DriveSupervisorCore()
    tick(core, 0.0, 0.1)
    assert tick(core, 0.05, 0.0) == (0.0, 0.0, core.IDLE)
    tick(core, 0.10, 0.1)
    assert tick(core, 0.15, 0.1, input_fresh=False) == (
        0.0, 0.0, core.IDLE)


def test_initial_direction_runs_alignment_sequence():
    core = DriveSupervisorCore()
    linear, _, state = tick(
        core, 0.0, 0.1, x=0.0, y=0.0, yaw=0.0)
    assert state == core.INITIAL_ALIGN
    assert linear > 0.0


def test_caster_alignment_applies_only_bounded_heading_trim():
    core = DriveSupervisorCore()
    tick(core, 0.0, 0.1, x=0.0, y=0.0, yaw=0.0)
    _, angular, state = tick(
        core, 0.05, 0.1, x=0.01, y=0.0, yaw=math.radians(8.0))
    assert state == core.INITIAL_ALIGN
    assert -0.05 <= angular < 0.0
    assert core.heading_error < 0.0


def test_reversal_runs_caster_sequence_then_resumes():
    core = DriveSupervisorCore(
        align_distance=0.10, initial_alignment=False)
    tick(core, 0.0, 0.1, x=0.0, y=0.0, yaw=0.0,
         measured_linear=0.1)
    linear, _, state = tick(
        core, 0.05, -0.1, x=0.0, y=0.0, yaw=0.0,
        measured_linear=0.1)
    assert state == core.BRAKE
    assert linear >= 0.0

    now = 0.05
    while core.state == core.BRAKE:
        now += 0.05
        tick(core, now, -0.1, x=0.0, y=0.0, yaw=0.0,
             measured_linear=0.0)
    assert core.state == core.ALIGN

    tick(core, now + 0.05, -0.1, x=-0.11, y=0.0, yaw=0.08,
         measured_linear=-0.06)
    assert core.state == core.ALIGN_STOP

    now += 0.05
    while core.state == core.ALIGN_STOP:
        now += 0.05
        tick(core, now, -0.1, x=-0.11, y=0.0, yaw=0.08,
             measured_linear=0.0)
    assert core.state == core.NORMAL
    assert core.straight_reference_yaw == 0.08
    linear, angular, _ = tick(
        core, now + 0.05, -0.1, x=-0.11, y=0.0, yaw=0.08,
        measured_linear=-0.01)
    assert linear < 0.0
    assert angular == 0.0

    assert core.last_drive_sign == -1

def test_post_align_accepts_new_heading_instead_of_returning_to_old_yaw():
    core = DriveSupervisorCore(initial_alignment=False)
    core.reversal_sign = 1
    core.reference_yaw = 0.0
    core.output_linear = 0.0
    core.output_angular = 0.0
    core.enter(core.ALIGN_STOP, 0.0)

    linear, angular, state = tick(
        core, 0.9, 0.1, x=0.12, y=0.0, yaw=math.radians(8.0),
        measured_linear=0.0)
    assert state == core.NORMAL
    assert linear == 0.0
    assert angular == 0.0
    assert core.straight_reference_yaw == math.radians(8.0)

    linear, angular, state = tick(
        core, 0.95, 0.1, x=0.12, y=0.0, yaw=math.radians(8.0),
        measured_linear=0.0)
    assert state == core.NORMAL
    assert linear > 0.0
    assert angular == 0.0


def test_reversal_is_cancelled_when_operator_releases_key():
    core = DriveSupervisorCore(initial_alignment=False)
    tick(core, 0.0, 0.1, yaw=0.0)
    tick(core, 0.05, -0.1, yaw=0.0)
    assert core.state == core.BRAKE
    assert tick(core, 0.10, 0.0, yaw=0.0) == (0.0, 0.0, core.IDLE)


def test_heading_wrap_chooses_short_rotation():
    core = DriveSupervisorCore()
    core.reference_yaw = math.radians(-179.0)
    command = core._heading_command(math.radians(179.0))
    assert 0.0 < command < core.heading_max


def test_straight_heading_hold_corrects_drift():
    core = DriveSupervisorCore(
        initial_alignment=False, heading_hold_ramp_time=0.0)
    tick(core, 0.0, 0.1, yaw=0.0)
    _, angular, state = tick(core, 0.05, 0.1, yaw=0.10)
    assert state == core.NORMAL
    assert angular < 0.0


def test_filtered_yaw_rate_damps_rotation():
    core = DriveSupervisorCore(
        initial_alignment=False, heading_hold_ramp_time=0.0,
        yaw_rate_filter_tau=0.0)
    tick(core, 0.0, 0.1, yaw=0.0)
    _, without_rate, _ = tick(
        core, 0.05, 0.1, yaw=math.radians(3.0), measured_angular=0.0)
    core.output_angular = 0.0
    _, with_rate, _ = tick(
        core, 0.10, 0.1, yaw=math.radians(3.0),
        measured_angular=math.radians(-5.0))
    assert with_rate > without_rate


def test_heading_hold_ramps_in_and_never_exceeds_limit():
    core = DriveSupervisorCore(
        initial_alignment=False, angular_accel=10.0,
        straight_heading_kp=2.0, straight_heading_max=0.05,
        heading_hold_ramp_time=1.0, yaw_rate_filter_tau=0.0)
    tick(core, 0.0, 0.1, yaw=0.0, measured_angular=0.0)
    _, early, _ = tick(
        core, 0.10, 0.1, yaw=math.radians(20.0), measured_angular=0.0)
    _, full, _ = tick(
        core, 1.10, 0.1, yaw=math.radians(20.0), measured_angular=0.0)
    assert -0.01 < early < 0.0
    assert full == -0.05


def test_intentional_turn_bypasses_heading_hold_and_recaptures():
    core = DriveSupervisorCore(initial_alignment=False)
    tick(core, 0.0, 0.1, yaw=0.0)
    _, angular, _ = tick(core, 0.05, 0.1, 0.2, yaw=0.05)
    assert angular > 0.0
    assert core.straight_reference_yaw is None
    tick(core, 0.10, 0.1, 0.0, yaw=0.08)
    assert core.straight_reference_yaw == 0.08
