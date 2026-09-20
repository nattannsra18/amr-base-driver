from amr_base_driver.safe_keyboard_teleop import DEFAULT_KEY_TIMEOUT
from amr_base_driver.safe_keyboard_teleop import adjusted_speed
from amr_base_driver.safe_keyboard_teleop import mapping_limits
from amr_base_driver.safe_keyboard_teleop import pivot_angular_speed


def test_mapping_caps_all_profiles():
    assert mapping_limits(.14, .70, True) == (.12, .18)
    assert mapping_limits(.04, .15, True) == (.08, .15)


def test_reverse_limits_and_stop():
    assert mapping_limits(-.14, -.70, True) == (-.12, -.18)
    assert mapping_limits(0, 0, True) == (0, 0)


def test_normal_driving_unchanged():
    assert mapping_limits(.14, .70, False) == (.14, .70)


def test_deadman_allows_default_desktop_key_repeat_to_start():
    assert DEFAULT_KEY_TIMEOUT > 0.50
    assert DEFAULT_KEY_TIMEOUT < 1.0


def test_mapping_pivot_clears_passive_caster_stiction_region():
    assert pivot_angular_speed(.20, True) == .60
    assert pivot_angular_speed(.70, True) == .70
    assert pivot_angular_speed(.35, False) == .35


def test_operator_speed_adjustment_is_bounded():
    assert adjusted_speed(.10, .01, .02, .20) == .11
    assert adjusted_speed(.20, .01, .02, .20) == .20
    assert adjusted_speed(.02, -.01, .02, .20) == .02
    assert adjusted_speed(.60, .05, .15, 1.00) == .65
    assert adjusted_speed(1.00, .05, .15, 1.00) == 1.00
