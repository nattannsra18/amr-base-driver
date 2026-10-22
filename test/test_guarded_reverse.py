import math
from pathlib import Path
import threading
from types import SimpleNamespace

from amr_base_driver.guarded_reverse import (
    DEFAULT_ARC_DISTANCE,
    DEFAULT_ARC_YAW,
    DEFAULT_WHEEL_SEPARATION,
    adjacent_cluster_clearance,
    arc_motion_complete,
    measured_arc_progress,
    rolling_pivot_turn_rate,
    normalize_angle,
    recovery_sector_clearances,
    robust_front_clearance,
    robust_rear_clearance,
    robust_turn_side_clearance,
    swept_arc_obstacle_distance,
    swept_straight_obstacle_distance,
    GuardedReverse,
)


def test_guarded_recovery_uses_two_executor_workers():
    source = (
        Path(__file__).resolve().parents[1] /
        'amr_base_driver' / 'guarded_reverse.py'
    ).read_text(encoding='utf-8')

    assert 'MultiThreadedExecutor(num_threads=2)' in source
    assert 'MultiThreadedExecutor(num_threads=3)' not in source


def test_guarded_recovery_sensor_streams_are_lazy(monkeypatch):
    subscriptions = []
    destroyed = []
    gate = SimpleNamespace(
        data_lock=threading.Lock(),
        latest_scan_geometry=None,
        scan_monotonic=0.0,
        latest_odom=None,
        latest_yaw=None,
        odom_monotonic=0.0,
        sensor_warmup_timeout=0.5,
        callbacks=object(),
        scan_callback=object(),
        odom_callback=object(),
        get_parameter=lambda name: SimpleNamespace(
            value='/scan' if name == 'scan_topic' else '/wheel/odometry'),
        create_subscription=lambda *args, **kwargs: (
            subscriptions.append((args, kwargs)) or object()),
        destroy_subscription=lambda subscription: destroyed.append(subscription),
    )

    monotonic_values = iter([0.0, 0.1, 0.6])
    monkeypatch.setattr(
        'amr_base_driver.guarded_reverse.time.monotonic',
        lambda: next(monotonic_values),
    )
    monkeypatch.setattr(
        'amr_base_driver.guarded_reverse.time.sleep', lambda _seconds: None)

    assert GuardedReverse.start_sensor_streams(gate) is False
    assert len(subscriptions) == 2

    GuardedReverse.stop_sensor_streams(gate)

    assert len(destroyed) == 2
    assert gate.scan_subscription is None
    assert gate.odom_subscription is None


def test_default_arc_distance_matches_rolling_pivot_geometry():
    assert math.isclose(
        DEFAULT_ARC_DISTANCE,
        0.5 * DEFAULT_WHEEL_SEPARATION * DEFAULT_ARC_YAW,
    )


def test_rolling_pivot_rate_stops_the_inner_wheel():
    speed = 0.05
    separation = 0.36094
    turn_rate = rolling_pivot_turn_rate(speed, separation)
    assert abs((-speed) + 0.5 * separation * turn_rate) < 1e-9


def scan(default=math.inf):
    return [default] * 360


def test_rear_clearance_ignores_one_invalid_speckle():
    ranges = scan()
    for degree in list(range(150, 180)) + list(range(180, 211)):
        ranges[degree % 360] = 1.2
    ranges[180] = 0.05
    clearance = robust_rear_clearance(
        ranges,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.10,
        range_max=8.0,
    )
    assert clearance == 1.2


def test_rear_clearance_detects_a_thin_adjacent_beam_obstacle():
    ranges = scan(2.0)
    for degree in range(178, 183):
        ranges[degree] = 0.22

    clearance = robust_rear_clearance(
        ranges,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.10,
        range_max=8.0,
    )

    assert clearance == 0.22


def test_rear_clearance_fails_closed_without_enough_returns():
    ranges = scan()
    ranges[180] = 2.0
    assert robust_rear_clearance(
        ranges,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.10,
        range_max=8.0,
    ) == 0.0


def test_front_clearance_detects_a_cluster_ahead():
    ranges = scan(2.0)
    ranges[0:3] = [0.55, 0.50, 0.55]

    assert robust_front_clearance(
        ranges,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.10,
        range_max=8.0,
    ) == 0.55


def test_turn_side_clearance_selects_requested_side_and_rear_corner():
    ranges = scan()
    for degree in range(70, 156):
        ranges[degree] = 0.9
    for degree in range(205, 291):
        ranges[degree] = 1.7

    common = {
        'angle_min': 0.0,
        'angle_increment': math.pi / 180.0,
        'range_min': 0.10,
        'range_max': 8.0,
    }
    assert robust_turn_side_clearance(
        ranges,
        turn_direction=1,
        **common,
    ) == 0.9
    assert robust_turn_side_clearance(
        ranges,
        turn_direction=-1,
        **common,
    ) == 1.7


def test_turn_side_clearance_fails_closed_without_enough_returns():
    ranges = scan()
    ranges[90] = 2.0
    assert robust_turn_side_clearance(
        ranges,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.10,
        range_max=8.0,
        turn_direction=1,
    ) == 0.0


def test_combined_sector_scan_matches_individual_clearances():
    ranges = scan()
    ranges[179:182] = [0.6, 0.5, 0.6]
    ranges[89:92] = [0.8, 0.7, 0.8]
    ranges[269:272] = [1.0, 0.9, 1.0]
    common = {
        'angle_min': 0.0,
        'angle_increment': math.pi / 180.0,
        'range_min': 0.10,
        'range_max': 8.0,
    }

    assert recovery_sector_clearances(ranges, **common) == (
        robust_rear_clearance(ranges, **common),
        robust_turn_side_clearance(ranges, turn_direction=1, **common),
        robust_turn_side_clearance(ranges, turn_direction=-1, **common),
    )


def test_adjacent_cluster_clearance_ignores_only_one_isolated_beam():
    assert adjacent_cluster_clearance([(4, 0.2), (9, 1.0)]) is None
    assert adjacent_cluster_clearance([(4, 0.2), (5, 0.21), (9, 1.0)]) == 0.21


def test_swept_arc_rejects_a_cluster_entering_the_rear_corner():
    ranges = scan()
    ranges[230] = 0.13
    ranges[231] = 0.13

    obstacle = swept_arc_obstacle_distance(
        ranges,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.10,
        range_max=8.0,
        turn_direction=1,
    )

    assert obstacle == 0.13


def test_swept_arc_ignores_one_isolated_collision_speckle():
    ranges = scan()
    ranges[227] = 0.13

    obstacle = swept_arc_obstacle_distance(
        ranges,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.10,
        range_max=8.0,
        turn_direction=1,
    )

    assert obstacle is None


def test_reverse_arc_allows_only_direction_moving_out_of_padding_overlap():
    ranges = scan()
    ranges[214] = 0.360
    ranges[215] = 0.354
    common = {
        'angle_min': -math.pi,
        'angle_increment': math.pi / 180.0,
        'range_min': 0.10,
        'range_max': 8.0,
        'motion_direction': -1,
    }

    assert swept_arc_obstacle_distance(
        ranges,
        turn_direction=-1,
        **common,
    ) is None
    assert swept_arc_obstacle_distance(
        ranges,
        turn_direction=1,
        **common,
    ) == 0.360


def test_swept_forward_arc_rejects_a_cluster_entering_front_corner():
    ranges = scan()
    ranges[27] = 0.34
    ranges[28] = 0.34

    obstacle = swept_arc_obstacle_distance(
        ranges,
        angle_min=0.0,
        angle_increment=math.pi / 180.0,
        range_min=0.10,
        range_max=8.0,
        turn_direction=1,
        motion_direction=1,
    )

    assert obstacle == 0.34


def test_straight_sweep_rejects_reverse_toward_a_rear_corner():
    ranges = scan()
    ranges[239] = 0.20
    ranges[240] = 0.20
    common = {
        'angle_min': 0.0,
        'angle_increment': math.pi / 180.0,
        'range_min': 0.10,
        'range_max': 8.0,
    }

    assert swept_straight_obstacle_distance(
        ranges,
        motion_direction=-1,
        **common,
    ) == 0.20
    assert swept_straight_obstacle_distance(
        ranges,
        motion_direction=1,
        **common,
    ) is None


def test_reverse_sweep_ignores_front_side_cluster_robot_moves_away_from():
    ranges = scan()
    ranges[145] = 0.353
    ranges[146] = 0.357
    common = {
        'angle_min': -math.pi,
        'angle_increment': math.pi / 180.0,
        'range_min': 0.10,
        'range_max': 8.0,
    }

    assert swept_straight_obstacle_distance(
        ranges,
        motion_direction=-1,
        **common,
    ) is None


def test_measured_arc_progress_handles_yaw_wrap_and_direction():
    start = (0.0, 0.0, math.radians(175.0))
    current = (-0.08, 0.02, math.radians(-165.0))
    travelled, directed_yaw = measured_arc_progress(start, current, 1)

    assert math.isclose(travelled, math.hypot(0.08, 0.02))
    assert math.isclose(directed_yaw, math.radians(20.0))
    assert math.isclose(normalize_angle(3.0 * math.pi), math.pi)


def test_arc_completion_requires_distance_and_yaw_in_requested_direction():
    start = (0.0, 0.0, 0.0)
    assert arc_motion_complete(
        start,
        (-0.11, 0.0, 0.31),
        turn_direction=1,
        target_distance=0.10,
        target_yaw=0.30,
    )
    assert not arc_motion_complete(
        start,
        (-0.11, 0.0, 0.20),
        turn_direction=1,
        target_distance=0.10,
        target_yaw=0.30,
    )
    assert not arc_motion_complete(
        start,
        (-0.11, 0.0, -0.31),
        turn_direction=1,
        target_distance=0.10,
        target_yaw=0.30,
    )


def test_arc_helpers_reject_invalid_turn_direction():
    ranges = scan(1.0)
    try:
        robust_turn_side_clearance(
            ranges,
            angle_min=0.0,
            angle_increment=math.pi / 180.0,
            range_min=0.10,
            range_max=8.0,
            turn_direction=0,
        )
    except ValueError as error:
        assert str(error) == 'turn_direction must be -1 or 1'
    else:
        raise AssertionError('invalid turn direction should fail')
