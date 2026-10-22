import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def navigation_after_readiness_exit(event, navigation_processes):
    """Release Nav2 only when the localization gate explicitly succeeded."""
    if event.returncode == 0:
        return [
            LogInfo(msg='Localization readiness succeeded; starting Nav2'),
            navigation_processes,
        ]
    return [LogInfo(msg=(
        'ERROR: Localization readiness gate exited without success '
        f'(code {event.returncode}); Nav2 remains stopped'))]


def generate_launch_description():
    base_share = get_package_share_directory('amr_base_driver')
    map_yaml = LaunchConfiguration('map_yaml')
    enable_motors = LaunchConfiguration('enable_motors')
    start_base_hardware = LaunchConfiguration('start_base_hardware')
    use_composition = LaunchConfiguration('use_composition')
    activate_after_initial_pose = LaunchConfiguration(
        'activate_after_initial_pose')
    start_localization = LaunchConfiguration('start_localization')
    start_navigation = LaunchConfiguration('start_navigation')
    ekf_transform_time_offset = LaunchConfiguration(
        'ekf_transform_time_offset')
    params_file = LaunchConfiguration('params_file')

    # A raw params-file substitution configures the top-level composable node,
    # but its internally-created costmap nodes do not receive their scoped
    # parameters. Normalize the file the same way as Nav2's Jazzy bringup so
    # /local_costmap/local_costmap and /global_costmap/global_costmap retain
    # the physical footprint and frame settings under composition.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'localization.launch.py')),
        launch_arguments={
            'map_yaml': map_yaml,
            'enable_motors': enable_motors,
            'start_base_hardware': start_base_hardware,
            'ekf_transform_time_offset': ekf_transform_time_offset,
            'start_localization': start_localization,
        }.items(),
    )

    common_parameters = [configured_params]
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=common_parameters,
        remappings=[('cmd_vel', '/cmd_vel_nav')],
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=common_parameters,
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=common_parameters,
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
        ],
    )
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=common_parameters,
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=common_parameters,
        remappings=[('cmd_vel', '/cmd_vel_nav')],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=[
            configured_params,
            {'default_nav_to_pose_bt_xml': os.path.join(
                base_share, 'behavior_trees',
                'navigate_to_pose_conservative.xml')},
        ],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=[{
            'autostart': False,
            # The ODROID-C4 can briefly miss a 4 s bond heartbeat while DDS
            # recovery work is active. Keep detection bounded without tearing
            # down a healthy stack during one transient scheduler stall.
            'bond_timeout': 10.0,
            'bond_heartbeat_period': 0.5,
            'node_names': [
                'controller_server',
                'planner_server',
                'velocity_smoother',
                'collision_monitor',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'velocity_smoother',
        'collision_monitor',
        'behavior_server',
        'bt_navigator',
    ]
    nav2_container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        # Match Nav2 Jazzy's supported composed bringup. The generic
        # multi-threaded container can race lifecycle preshutdown callbacks
        # and abort while systemd is stopping or restarting navigation.
        executable='component_container_isolated',
        output='screen',
        condition=IfCondition(use_composition),
        # Load the complete parameter file into the container process. The
        # LoadNode service only forwards parameters matching the top-level
        # component name, while controller_server and planner_server create
        # their costmap nodes internally.
        parameters=[configured_params],
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=common_parameters,
                remappings=[('cmd_vel', '/cmd_vel_nav')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=common_parameters,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=common_parameters,
                remappings=[
                    ('cmd_vel', '/cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=common_parameters,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=common_parameters,
                remappings=[('cmd_vel', '/cmd_vel_nav')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[
                    configured_params,
                    {'default_nav_to_pose_bt_xml': os.path.join(
                        base_share,
                        'behavior_trees',
                        'navigate_to_pose_conservative.xml')},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'autostart': False,
                    'bond_timeout': 10.0,
                    'bond_heartbeat_period': 0.5,
                    'node_names': lifecycle_nodes,
                }],
            ),
        ],
    )
    activation_gate = Node(
        package='amr_base_driver',
        executable='nav2_activation_gate',
        name='nav2_activation_gate',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", start_navigation, "' == 'true' and '",
            activate_after_initial_pose, "' == 'true'",
        ])),
    )
    localization_ready = Node(
        package='amr_base_driver',
        executable='localization_readiness_gate',
        name='navigation_process_readiness_gate',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", start_navigation, "' == 'true' and '",
            start_localization, "' == 'true'",
        ])),
        parameters=[{'request_startup': False}],
    )
    navigation_processes = GroupAction(actions=[
        GroupAction(
            condition=UnlessCondition(use_composition),
            actions=[
                controller,
                planner,
                velocity_smoother,
                collision_monitor,
                behavior_server,
                bt_navigator,
                lifecycle_manager,
            ],
        ),
        nav2_container,
        activation_gate,
    ])
    start_navigation_when_localized = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_ready,
            on_exit=lambda event, _context: navigation_after_readiness_exit(
                event, navigation_processes),
        )
    )

    default_params = os.path.join(
        base_share, 'config', 'nav2_physical.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml', default_value='',
            description='Absolute path to the selected physical map YAML.'),
        DeclareLaunchArgument(
            'start_localization', default_value='true',
            description='Start AMCL only when an active map is available.'),
        DeclareLaunchArgument(
            'start_navigation', default_value='true',
            description='Start Nav2 only when an active map is available.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Physical-robot Nav2 parameter file.'),
        DeclareLaunchArgument(
            'enable_motors', default_value='false',
            description=(
                'Keep false for costmap and planning validation. Enable only '
                'for an attended floor test with the motor cutoff ready.')),
        DeclareLaunchArgument(
            'start_base_hardware', default_value='true',
            description=(
                'Start base hardware inside this launch. The systemd boot '
                'deployment uses its dedicated critical service instead.')),
        DeclareLaunchArgument(
            'use_composition', default_value='false',
            description=(
                'Run the Nav2 C++ nodes in one multi-threaded component '
                'container to reduce DDS participants. Keep false on the '
                'ODROID until composed shutdown no longer hangs.')),
        DeclareLaunchArgument(
            'activate_after_initial_pose', default_value='true',
            description=(
                'Start navigation lifecycle only after localization publishes '
                'the map to base transform.')),
        DeclareLaunchArgument(
            'ekf_transform_time_offset', default_value='0.10',
            description=(
                'Future offset for odom to base TF. This applies only to '
                'localization/Nav2; mapping keeps the EKF default of zero.')),
        localization,
        # Start the heavier Nav2 process group only after map_server and AMCL
        # are active. This is lifecycle readiness, not a hardware-specific
        # fixed delay.
        start_navigation_when_localized,
        localization_ready,
    ])
