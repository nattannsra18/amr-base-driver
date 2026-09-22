import math

from amr_base_driver.scan_resampler import resample_ranges


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
