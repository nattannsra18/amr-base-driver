import math

from sensor_msgs.msg import LaserScan

from amr_base_driver.scan_resampler import (
    has_valid_scan_metadata,
    resample_ranges,
)


def test_cardinal_angles_are_placed_in_expected_bins():
    values = [1.0, 2.0, 3.0, 4.0]
    output = resample_ranges(
        -math.pi, math.pi / 2.0, values, 360, 0.1, 12.0)
    assert output[0] == 1.0
    assert output[90] == 2.0
    assert output[180] == 3.0
    assert output[270] == 4.0


def test_invalid_ranges_remain_infinite():
    output = resample_ranges(
        -math.pi, math.pi / 2.0,
        [0.0, math.nan, 13.0, math.inf], 360, 0.1, 12.0)
    assert all(math.isinf(value) for value in output)


def test_multiple_samples_in_bin_keep_nearest_obstacle():
    output = resample_ranges(
        -math.pi, math.radians(0.2), [2.0, 1.0], 360, 0.1, 12.0)
    assert output[0] == 1.0


def test_nonfinite_angles_are_rejected_without_crashing():
    for angle_min, angle_increment in (
            (math.nan, math.radians(1.0)),
            (-math.pi, math.nan)):
        output = resample_ranges(
            angle_min, angle_increment, [1.0], 360, 0.1, 12.0)
        assert all(math.isinf(value) for value in output)


def test_scan_metadata_validation_rejects_malformed_frames():
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = math.radians(1.0)
    scan.range_min = 0.1
    scan.range_max = 12.0
    scan.scan_time = 0.1
    assert has_valid_scan_metadata(scan)

    invalid_values = (
        ('angle_min', math.nan),
        ('angle_increment', math.nan),
        ('angle_increment', 0.0),
        ('range_min', math.nan),
        ('range_max', math.nan),
        ('scan_time', math.nan),
    )
    for attribute, value in invalid_values:
        malformed = LaserScan()
        malformed.angle_min = scan.angle_min
        malformed.angle_increment = scan.angle_increment
        malformed.range_min = scan.range_min
        malformed.range_max = scan.range_max
        malformed.scan_time = scan.scan_time
        setattr(malformed, attribute, value)
        assert not has_valid_scan_metadata(malformed)
