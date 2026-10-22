import math

from amr_web_bridge.stall_escape import (
    choose_stall_arcs,
    choose_stall_escape,
    choose_stall_turns,
)


def scan_with(*, front=0.2, rear=0.2, left=0.2, right=0.2):
    values = [0.2] * 360
    for degrees in range(360):
        signed = degrees if degrees <= 180 else degrees - 360
        if abs(signed) <= 30:
            values[degrees] = front
        elif abs(signed) >= 150:
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


def test_prefers_straight_reverse_over_arc_when_rear_is_clear():
    decision = choose(scan_with(rear=0.8, left=1.2, right=1.0))
    assert decision.action == 'back_up'
    assert decision.value == 0.10


def test_does_not_auto_pivot_when_reverse_and_arc_are_blocked():
    assert choose(scan_with(rear=0.25, left=0.9, right=0.3)).action == 'none'
    assert choose(scan_with(rear=0.25, left=0.3, right=0.9)).action == 'none'


def test_uses_forward_arc_when_reverse_is_blocked_but_front_left_is_clear():
    decision = choose(scan_with(
        front=0.54,
        rear=0.29,
        left=0.57,
        right=0.38,
    ))

    assert decision.action == 'forward_arc'
    assert decision.value == 0.22
    assert 'rolling forward left arc' in decision.detail


def test_moves_forward_when_a_rear_corner_is_close():
    decision = choose(scan_with(
        front=0.69,
        rear=0.30,
        left=0.94,
        right=0.14,
    ))

    assert decision.action == 'forward'
    assert decision.value == 0.10
    assert 'moving away from close side' in decision.detail


def test_forward_arc_fails_closed_when_front_is_too_close():
    decision = choose(scan_with(
        front=0.44,
        rear=0.29,
        left=0.80,
        right=0.80,
    ))

    assert decision.action == 'none'


def test_refuses_motion_when_no_sector_is_safe_or_scan_is_invalid():
    assert choose(scan_with(rear=0.2, left=0.3, right=0.3)).action == 'none'
    assert choose([float('inf')] * 360).action == 'none'


def test_single_speckle_does_not_hide_an_otherwise_clear_sector():
    values = scan_with(rear=0.8, left=0.39, right=0.39)
    values[180] = 0.06
    assert choose(values).action == 'back_up'


def test_thin_multi_beam_obstacle_blocks_the_rear_sector():
    values = scan_with(rear=0.8, left=0.39, right=0.39)
    for degree in range(178, 183):
        values[degree] = 0.22

    assert choose(values).action == 'none'


def test_rolling_arc_is_preferred_to_pure_spin_when_short_rear_is_safe():
    decision = choose(scan_with(rear=0.40, left=0.75, right=0.55))

    assert decision.action == 'arc'
    assert decision.value == 0.22
    assert 'rolling reverse left arc' in decision.detail


def test_arc_options_fail_closed_when_rear_is_too_close():
    arcs = choose_stall_arcs(
        scan_with(rear=0.28, left=0.9, right=0.9),
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.05,
        range_max=8.0,
    )

    assert arcs == ()


def test_turn_options_include_both_safe_sides_in_clearance_order():
    turns = choose_stall_turns(
        scan_with(rear=0.8, left=0.7, right=1.1),
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.05,
        range_max=8.0,
    )

    assert [turn.value for turn in turns] == [-0.26, 0.26]
    assert turns[0].clearance == 1.1


def test_turn_options_exclude_a_scan_blocked_side():
    turns = choose_stall_turns(
        scan_with(rear=0.8, left=0.3, right=0.9),
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.05,
        range_max=8.0,
    )

    assert [turn.value for turn in turns] == [-0.26]
