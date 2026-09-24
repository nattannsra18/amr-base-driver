import math

from amr_web_bridge.stall_escape import choose_stall_escape


def scan_with(*, rear=0.2, left=0.2, right=0.2):
    values = [0.2] * 360
    for degrees in range(360):
        signed = degrees if degrees <= 180 else degrees - 360
        if abs(signed) >= 150:
            values[degrees] = rear
        elif 50 <= signed <= 130:
            values[degrees] = left
        elif -130 <= signed <= -50:
            values[degrees] = right
    return values


def choose(values):
    return choose_stall_escape(
        values,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.05,
        range_max=8.0,
    )


def test_prefers_short_collision_checked_reverse_when_rear_is_clear():
    decision = choose(scan_with(rear=0.8, left=1.2, right=1.0))
    assert decision.action == 'back_up'
    assert decision.value == 0.10


def test_turns_toward_clearer_side_when_reverse_is_blocked():
    assert choose(scan_with(rear=0.25, left=0.9, right=0.3)).value > 0
    assert choose(scan_with(rear=0.25, left=0.3, right=0.9)).value < 0


def test_refuses_motion_when_no_sector_is_safe_or_scan_is_invalid():
    assert choose(scan_with(rear=0.2, left=0.3, right=0.3)).action == 'none'
    assert choose([float('inf')] * 360).action == 'none'


def test_single_speckle_does_not_hide_an_otherwise_clear_sector():
    values = scan_with(rear=0.8, left=0.2, right=0.2)
    values[180] = 0.06
    assert choose(values).action == 'back_up'
