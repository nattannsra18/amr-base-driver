from pathlib import Path
import xml.etree.ElementTree as ET

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


def test_ekf_rate_is_bounded_for_odroid_sensor_cadence():
    params_path = PACKAGE_ROOT / 'config' / 'ekf.yaml'
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    assert params['ekf_filter_node']['ros__parameters']['frequency'] == 15.0


def test_tested_nav2_profile_keeps_default_bond_cadence():
    params = load_nav2_params()
    for name in (
        'controller_server', 'planner_server', 'bt_navigator',
        'behavior_server', 'velocity_smoother', 'collision_monitor',
    ):
        assert 'bond_heartbeat_period' not in params[name]['ros__parameters']

    localization = yaml.safe_load((
        PACKAGE_ROOT / 'config' / 'localization.yaml'
    ).read_text(encoding='utf-8'))
    for name in ('map_server', 'amcl'):
        assert localization[name]['ros__parameters'][
            'bond_heartbeat_period'
        ] == 0.5


def test_imu_conditioning_is_integrated_without_dds_republish_process():
    source = (
        PACKAGE_ROOT / 'amr_base_driver' / 'serial_bridge.py'
    ).read_text(encoding='utf-8')
    launch = (
        PACKAGE_ROOT / 'launch' / 'base_hardware.launch.py'
    ).read_text(encoding='utf-8')
    assert 'self.imu_bias_estimator = StationaryBias()' in source
    assert "Imu, 'imu/data', qos_profile_sensor_data" in source
    assert "executable='imu_conditioner'" not in launch


def test_web_bridge_reduces_idle_polling_without_relaxing_deadman():
    source = (
        PACKAGE_ROOT / 'robot_agent' / 'amr_web_bridge' /
        'amr_web_bridge' / 'web_bridge_node.py'
    ).read_text(encoding='utf-8')
    assert "'work': 0.2" in source
    assert "'mapping_deadman': 0.1" in source
    assert "0.25 if status.get('phase') == 'MAPPING' else 1.0" in source


def test_lidar_driver_recovers_after_parser_crash():
    for launch_name in ('localization.launch.py', 'mapping.launch.py'):
        launch = (PACKAGE_ROOT / 'launch' / launch_name).read_text(
            encoding='utf-8',
        )
        assert "name='ydlidar_ros2_driver_node'" in launch
        assert 'respawn=True' in launch
        assert 'respawn_delay=5.0' in launch


def test_mapping_teleop_accepts_only_bounded_manual_speeds():
    bridge = (
        PACKAGE_ROOT / 'robot_agent' / 'amr_web_bridge' / 'amr_web_bridge' /
        'web_bridge_node.py'
    ).read_text(encoding='utf-8')
    assert 'abs(float(linear_x)) <= 0.30' in bridge
    assert 'abs(float(angular_z)) <= 1.0' in bridge


def test_mapping_start_requires_a_fresh_lidar_scan():
    bridge = (
        PACKAGE_ROOT / 'robot_agent' / 'amr_web_bridge' / 'amr_web_bridge' /
        'web_bridge_node.py'
    ).read_text(encoding='utf-8')
    assert 'time.monotonic() - last_scan > 1.0' in bridge
    assert 'Fresh LiDAR scan is unavailable' in bridge


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
    assert params['controller_server']['ros__parameters']['odom_topic'] == (
        '/odometry/filtered'
    )
    controller = params['controller_server']['ros__parameters']['FollowPath']
    assert controller['plugin'].endswith('RegulatedPurePursuitController')
    assert controller['desired_linear_vel'] == 0.15
    assert controller['rotate_to_heading_angular_vel'] == 0.60
    assert controller['max_angular_accel'] == 6.0
    assert controller['allow_reversing'] is False
    assert controller['use_rotate_to_heading'] is True
    assert controller['lookahead_dist'] == 0.30
    assert controller['min_lookahead_dist'] == 0.20
    assert controller['max_lookahead_dist'] == 0.45
    assert controller['use_velocity_scaled_lookahead_dist'] is True
    assert 'use_fixed_curvature_lookahead' not in controller
    assert controller['cost_scaling_dist'] == 0.35
    assert controller['cost_scaling_gain'] == 1.0
    assert controller['inflation_cost_scaling_factor'] == 5.0


def test_progress_checker_uses_tested_translation_progress_contract():
    params = load_nav2_params()['controller_server']['ros__parameters']
    checker = params['progress_checker']
    assert checker['plugin'] == 'nav2_controller::SimpleProgressChecker'
    assert checker['required_movement_radius'] == 0.05
    assert 'required_movement_angle' not in checker
    assert checker['movement_time_allowance'] == 15.0
    assert params['failure_tolerance'] == 0.5


def test_delivery_goal_uses_tested_pose_tolerances():
    checker = load_nav2_params()['controller_server']['ros__parameters'][
        'goal_checker'
    ]
    assert checker['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert checker['xy_goal_tolerance'] == 0.08
    assert checker['yaw_goal_tolerance'] == 0.15

    profile = yaml.safe_load((
        PACKAGE_ROOT / 'robot_agent' / 'amr_web_bridge' / 'config' /
        'profiles' / 'odroid_c4_tt_prototype.yaml'
    ).read_text(encoding='utf-8'))['amr_web_bridge']['ros__parameters']
    assert profile['near_goal_acceptance_distance_m'] == 0.20
    assert profile['near_goal_acceptance_requires_heading'] is False
    assert profile['enable_blocked_pose_motion_recovery'] is True
    assert profile['max_blocked_pose_recovery_attempts'] == 1


def test_tested_navfn_planner_and_inflation_are_frozen():
    params = load_nav2_params()
    planner = params['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    assert planner['use_astar'] is True
    assert planner['allow_unknown'] is False
    for name in ('local_costmap', 'global_costmap'):
        inflation = params[name][name]['ros__parameters']['inflation_layer']
        assert inflation['inflation_radius'] == 0.35
        assert inflation['cost_scaling_factor'] == 5.0


def test_progress_checker_avoids_four_second_false_aborts():
    params = load_nav2_params()
    progress_checker = (
        params['controller_server']['ros__parameters']['progress_checker']
    )
    assert progress_checker['required_movement_radius'] == 0.05
    assert progress_checker['movement_time_allowance'] == 15.0


def test_behavior_server_exposes_only_bounded_recovery_plugins():
    params = load_nav2_params()
    behavior = params['behavior_server']['ros__parameters']
    assert behavior['behavior_plugins'] == ['backup', 'wait']
    assert behavior['robot_base_frame'] == 'base_footprint'
    assert behavior['simulate_ahead_time'] == 2.0
    assert 'max_rotational_vel' not in behavior


def test_collision_monitor_looks_ahead_before_the_physical_footprint_arrives():
    monitor = load_nav2_params()['collision_monitor']['ros__parameters']
    approach = monitor['FootprintApproach']
    assert approach['action_type'] == 'approach'
    assert approach['time_before_collision'] == 1.5
    assert approach['footprint_topic'] == '/local_costmap/published_footprint'


def test_behavior_tree_keeps_tested_bounded_backup_recovery():
    tree_path = (
        PACKAGE_ROOT / 'behavior_trees' /
        'navigate_to_pose_conservative.xml'
    )
    tree = tree_path.read_text(encoding='utf-8')
    assert '<FollowPath' in tree
    assert '<Spin' not in tree
    assert '<BackUp backup_dist="0.12"' in tree
    assert 'name="NavigateRecovery"' in tree
    assert 'name="ClearBothCostmaps"' in tree
    assert '<Spin' not in tree


def test_behavior_tree_replans_at_the_tested_one_hz_rate():
    tree_path = (
        PACKAGE_ROOT / 'behavior_trees' /
        'navigate_to_pose_conservative.xml'
    )
    pipeline = ET.parse(tree_path).find(
        ".//PipelineSequence[@name='NavigateWithConservativeReplanning']"
    )
    assert pipeline is not None
    rate_controller = pipeline.find('./RateController')
    assert rate_controller is not None
    assert rate_controller.get('hz') == '1.0'
    assert rate_controller.find('.//IsPathValid') is not None
    assert rate_controller.find('.//ComputePathToPose') is not None


def test_navigation_launch_starts_and_manages_behavior_server():
    launch_path = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
    launch = launch_path.read_text(encoding='utf-8')
    assert "package='nav2_behaviors'" in launch
    assert "executable='behavior_server'" in launch
    assert "remappings=[('cmd_vel', '/cmd_vel_nav')]" in launch
    assert "'bond_timeout': 10.0" in launch
    assert "'behavior_server'," in launch


def test_navigation_keeps_composition_as_an_opt_in_ab_path():
    launch = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
        encoding='utf-8')

    assert "'use_composition', default_value='false'" in launch
    assert "executable='component_container_isolated'" in launch
    assert "plugin='nav2_controller::ControllerServer'" in launch
    assert "plugin='nav2_planner::PlannerServer'" in launch
    assert "plugin='nav2_lifecycle_manager::LifecycleManager'" in launch
    assert 'UnlessCondition(use_composition)' in launch
    assert 'ParameterFile(' in launch
    assert 'RewrittenYaml(' in launch
    assert 'common_parameters = [configured_params]' in launch
    assert 'parameters=[configured_params]' in launch


def test_nav2_common_is_declared_for_parameter_rewriting():
    package_xml = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
    assert '<exec_depend>nav2_common</exec_depend>' in package_xml


def test_reverse_ab_profile_is_isolated_from_production_defaults():
    candidate = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_bidirectional_ab.yaml').read_text(
            encoding='utf-8'))
    planner = candidate['planner_server']['ros__parameters']
    controller = candidate['controller_server']['ros__parameters']
    assert planner['planner_plugins'] == ['Bidirectional']
    assert planner['Bidirectional']['allow_reverse_expansion'] is True
    assert '/diff/output.json' in planner['Bidirectional']['lattice_filepath']
    assert controller['ReverseFollowPath']['allow_reversing'] is True
    assert controller['ReverseFollowPath']['use_rotate_to_heading'] is False
    production = load_nav2_params()['controller_server']['ros__parameters']
    assert production['controller_plugins'] == ['FollowPath']
    assert production['FollowPath']['allow_reversing'] is False


def test_localization_waits_for_lifecycle_bonds_under_startup_load():
    launch = (PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text(
        encoding='utf-8')
    assert "'bond_timeout': 10.0" in launch
    assert "'autostart': False" in launch
    assert "executable='localization_readiness_gate'" in launch


def test_navigation_processes_wait_for_localization_readiness():
    launch = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
        encoding='utf-8')
    assert 'OnProcessExit(' in launch
    assert 'target_action=localization_ready' in launch
    assert 'navigation_after_readiness_exit(' in launch
    assert 'if event.returncode == 0:' in launch
    assert 'Nav2 remains stopped' in launch
    assert "'request_startup': False" in launch


def test_navigation_activation_condition_quotes_launch_values():
    launch = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
        encoding='utf-8')
    # LaunchConfiguration values substitute to bare words such as `true`.
    # PythonExpression must quote them, otherwise it evaluates a nonexistent
    # Python name and tears down the entire navigation launch at the timer.
    assert '"\'", start_navigation, "\' == \'true\' and \'"' in launch
    assert 'activate_after_initial_pose, "\' == \'true\'"' in launch


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
    assert 'ParameterValue(' in base_hardware


def test_supported_launch_path_has_one_command_arbiter_and_no_fixed_delays():
    base_hardware = (
        PACKAGE_ROOT / 'launch' / 'base_hardware.launch.py'
    ).read_text(encoding='utf-8')
    localization = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py'
    ).read_text(encoding='utf-8')
    mapping = (
        PACKAGE_ROOT / 'launch' / 'mapping.launch.py'
    ).read_text(encoding='utf-8')

    assert "executable='cmd_vel_passthrough'" in base_hardware
    assert "executable='drive_supervisor'" not in base_hardware
    assert 'enable_drive_supervisor' not in base_hardware
    assert 'TimerAction' not in localization
    assert 'TimerAction' not in mapping


def test_low_rate_observability_streams_reduce_odroid_dds_load():
    base_hardware = (
        PACKAGE_ROOT / 'launch' / 'base_hardware.launch.py'
    ).read_text(encoding='utf-8')
    localization = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py'
    ).read_text(encoding='utf-8')
    mapping = (
        PACKAGE_ROOT / 'launch' / 'mapping.launch.py'
    ).read_text(encoding='utf-8')

    assert "parameters=[{'max_topic_hz': 1.0}]" in base_hardware
    assert "'telemetry_hz': 2.0" in localization
    assert "'telemetry_hz': 2.0" in mapping


def test_guarded_recovery_uses_the_full_rate_safety_scan():
    launch = (
        PACKAGE_ROOT / 'launch' / 'base_hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "parameters=[{'scan_topic': '/scan'}]" in launch
