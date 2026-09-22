import math

from amr_base_driver.guarded_reverse import robust_rear_clearance


def scan(default=math.inf):
    return [default] * 360


def test_rear_clearance_uses_rear_sector_tenth_percentile():
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
