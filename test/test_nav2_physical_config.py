from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_FOOTPRINT = [
    [0.303, 0.190],
    [0.303, -0.190],
    [-0.087, -0.190],
    [-0.087, 0.190],
]


def load_nav2_params():
    params_path = PACKAGE_ROOT / 'config' / 'nav2_physical.yaml'
    return yaml.safe_load(params_path.read_text(encoding='utf-8'))


def test_local_and_global_costmaps_use_measured_footprint():
    params = load_nav2_params()
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        assert yaml.safe_load(costmap['footprint']) == EXPECTED_FOOTPRINT
        assert costmap['footprint_padding'] == 0.02


def test_initial_velocity_and_acceleration_limits_are_conservative():
    params = load_nav2_params()
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [0.10, 0.0, 0.60]
    assert smoother['max_accel'] == [0.10, 0.0, 0.40]
    assert smoother['max_decel'] == [-0.15, 0.0, -0.50]


def test_controller_uses_regulated_pure_pursuit_without_reversing():
    params = load_nav2_params()
    controller = params['controller_server']['ros__parameters']['FollowPath']
    assert controller['plugin'].endswith('RegulatedPurePursuitController')
    assert controller['desired_linear_vel'] == 0.08
    assert controller['rotate_to_heading_angular_vel'] == 0.60
    assert controller['max_angular_accel'] == 6.0
    assert controller['allow_reversing'] is False


def test_behavior_server_only_enables_conservative_recovery_plugins():
    params = load_nav2_params()
    behavior = params['behavior_server']['ros__parameters']
    assert behavior['behavior_plugins'] == ['backup', 'wait']
    assert behavior['robot_base_frame'] == 'base_footprint'
    assert behavior['simulate_ahead_time'] == 2.0


def test_behavior_tree_uses_short_backup_without_automatic_spin():
    tree_path = (
        PACKAGE_ROOT / 'behavior_trees' /
        'navigate_to_pose_conservative.xml'
    )
    tree = tree_path.read_text(encoding='utf-8')
    assert '<FollowPath' in tree
    assert '<Spin' not in tree
    assert '<BackUp backup_dist="0.12"' in tree
    assert 'backup_speed="0.08"' in tree
    assert 'time_allowance="4.0"' in tree


def test_navigation_launch_starts_and_manages_behavior_server():
    launch_path = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
    launch = launch_path.read_text(encoding='utf-8')
    assert "package='nav2_behaviors'" in launch
    assert "executable='behavior_server'" in launch
    assert "remappings=[('cmd_vel', '/cmd_vel_nav')]" in launch
    assert "'behavior_server'," in launch


def test_navigation_applies_tf_offset_without_changing_mapping_default():
    navigation = (
        PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
    ).read_text(encoding='utf-8')
    localization = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py'
    ).read_text(encoding='utf-8')
    base_hardware = (
        PACKAGE_ROOT / 'launch' / 'base_hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "'ekf_transform_time_offset', default_value='0.10'" in navigation
    assert "'ekf_transform_time_offset': ekf_transform_time_offset" in navigation
    assert "'ekf_transform_time_offset', default_value='0.10'" in localization
    assert "'ekf_transform_time_offset': ekf_transform_time_offset" in localization
    assert "'ekf_transform_time_offset', default_value='0.0'" in base_hardware
    assert "ParameterValue(" in base_hardware
