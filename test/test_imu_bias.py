import math
from amr_base_driver.imu_bias import StationaryBias


def test_learns_bias_with_measured_cross_axis_noise():
    b = StationaryBias()
    for i in range(140):
        b.update(i*.05, [.022*math.sin(i), .019*math.sin(i*.8), .001], 9.5, True)
    assert b.ready
    assert abs(b.bias[2]-.001) < 1e-6


def test_does_not_learn_during_motion():
    b = StationaryBias()
    for i in range(200):
        b.update(i*.05, [0, 0, .001], 9.8, False)
    assert not b.ready


def test_learns_measured_temperature_dependent_boot_offset():
    b = StationaryBias()
    for i in range(140):
        gyro = [
            .0507 + .0035*math.sin(i),
            -.0016 + .0065*math.sin(i*.7),
            -.0496 + .0155*math.sin(i*.9),
        ]
        b.update(i*.05, gyro, 9.50, True)
    assert b.ready
    assert abs(b.bias[0]-.0507) < .001
    assert abs(b.bias[2]+.0496) < .001


def test_rejects_large_rotation():
    b = StationaryBias()
    for i in range(200):
        b.update(i*.05, [0, 0, .15], 9.8, True)
    assert not b.ready
