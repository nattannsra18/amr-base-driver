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


def test_local_and_global_costmaps_use_measured_footprint():
    params = load_nav2_params()
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        assert yaml.safe_load(costmap['footprint']) == EXPECTED_FOOTPRINT
        assert costmap['footprint_padding'] == 0.02


def test_initial_velocity_and_acceleration_limits_are_conservative():
    params = load_nav2_params()
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [0.30, 0.0, 1.20]
    assert smoother['max_accel'] == [0.10, 0.0, 0.40]
    assert smoother['max_decel'] == [-0.15, 0.0, -0.50]


def test_controller_uses_regulated_pure_pursuit_without_reversing():
    params = load_nav2_params()
    assert params['controller_server']['ros__parameters']['odom_topic'] == (
        '/odometry/filtered'
    )
    controller = params['controller_server']['ros__parameters']['FollowPath']
    assert controller['plugin'].endswith('RegulatedPurePursuitController')
    assert controller['desired_linear_vel'] == 0.30
    assert controller['rotate_to_heading_angular_vel'] == 1.20
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


def test_progress_checker_allows_caster_and_static_friction_settle_time():
    params = load_nav2_params()['controller_server']['ros__parameters']
    checker = params['progress_checker']
    assert checker['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert checker['required_movement_radius'] == 0.05
    assert checker['required_movement_angle'] == 0.15
    assert checker['movement_time_allowance'] == 15.0
    assert params['failure_tolerance'] == 0.5


def test_delivery_goal_is_an_area_without_required_final_heading():
    checker = load_nav2_params()['controller_server']['ros__parameters'][
        'goal_checker'
    ]
    assert checker['plugin'] == 'nav2_controller::PositionGoalChecker'
    assert checker['xy_goal_tolerance'] == 0.20
    assert checker['path_length_tolerance'] == 0.75
    assert 'yaw_goal_tolerance' not in checker

    profile = yaml.safe_load((
        PACKAGE_ROOT / 'robot_agent' / 'amr_web_bridge' / 'config' /
        'profiles' / 'odroid_c4_tt_prototype.yaml'
    ).read_text(encoding='utf-8'))['amr_web_bridge']['ros__parameters']
    assert profile['near_goal_acceptance_distance_m'] == 0.20
    assert profile['near_goal_acceptance_requires_heading'] is False
    assert profile['enable_blocked_pose_motion_recovery'] is False


def test_smac_planner_uses_differential_drive_paths_without_car_turn_radius():
    params = load_nav2_params()
    planner = params['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert planner['use_final_approach_orientation'] is True
    assert planner['cost_travel_multiplier'] == 2.0
    assert 'motion_model_for_search' not in planner
    assert 'minimum_turning_radius' not in planner
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
    # Spin is available to the scan-gated one-shot stall recovery, while the
    # default behavior tree below still forbids an automatic blind spin.
    assert behavior['behavior_plugins'] == ['backup', 'spin', 'wait']
    assert behavior['robot_base_frame'] == 'base_footprint'
    assert behavior['simulate_ahead_time'] == 2.0
    assert behavior['max_rotational_vel'] == 0.50
    assert behavior['min_rotational_vel'] == 0.50
    assert behavior['rotational_acc_lim'] == 0.35


def test_collision_monitor_looks_ahead_before_the_physical_footprint_arrives():
    monitor = load_nav2_params()['collision_monitor']['ros__parameters']
    approach = monitor['FootprintApproach']
    assert approach['action_type'] == 'approach'
    assert approach['time_before_collision'] == 2.0
    assert approach['footprint_topic'] == '/local_costmap/published_footprint'


def test_behavior_tree_leaves_motion_recovery_to_scan_guarded_agent():
    tree_path = (
        PACKAGE_ROOT / 'behavior_trees' /
        'navigate_to_pose_conservative.xml'
    )
    tree = tree_path.read_text(encoding='utf-8')
    assert '<FollowPath' in tree
    assert '<Spin' not in tree
    assert '<BackUp' not in tree
    assert 'name="ComputePathRecovery"' in tree
    assert 'ClearGlobalCostmapForPlanner' in tree
    assert 'name="FollowPathRecovery"' not in tree
    assert 'ClearLocalCostmapForController' not in tree
    assert 'name="NavigateRecovery"' not in tree


def test_behavior_tree_rate_limits_the_complete_planner_recovery_branch():
    tree_path = (
        PACKAGE_ROOT / 'behavior_trees' /
        'navigate_to_pose_conservative.xml'
    )
    pipeline = ET.parse(tree_path).find(
        ".//PipelineSequence[@name='NavigateWithConservativeReplanning']"
    )
    assert pipeline is not None
    rate_controller = pipeline.find(
        "./RateController[@name='RateControllerComputePathToPose']"
    )
    assert rate_controller is not None
    assert rate_controller.get('hz') == '1.0'
    recovery = rate_controller.find(
        "./RecoveryNode[@name='ComputePathRecovery']"
    )
    assert recovery is not None
    assert recovery.find('./ComputePathToPose') is not None
    assert recovery.find('.//IsPathValid') is None


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
    assert 'on_exit=[navigation_processes]' in launch
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
