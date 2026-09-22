import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    base_share = get_package_share_directory('amr_base_driver')
    map_yaml = LaunchConfiguration('map_yaml')
    enable_motors = LaunchConfiguration('enable_motors')
    activate_after_initial_pose = LaunchConfiguration(
        'activate_after_initial_pose')
    start_localization = LaunchConfiguration('start_localization')
    start_navigation = LaunchConfiguration('start_navigation')
    ekf_transform_time_offset = LaunchConfiguration(
        'ekf_transform_time_offset')
    params_file = LaunchConfiguration('params_file')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'localization.launch.py')),
        launch_arguments={
            'map_yaml': map_yaml,
            'enable_motors': enable_motors,
            'enable_drive_supervisor': 'false',
            'ekf_transform_time_offset': ekf_transform_time_offset,
            'start_localization': start_localization,
        }.items(),
    )

    common_parameters = [params_file]
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
    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
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
            params_file,
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
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'velocity_smoother',
                'collision_monitor',
                'behavior_server',
                'bt_navigator',
            ],
        }],
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
        TimerAction(
            period=12.0,
            actions=[
                controller,
                planner,
                smoother,
                velocity_smoother,
                collision_monitor,
                behavior_server,
                bt_navigator,
                lifecycle_manager,
                activation_gate,
            ],
        ),
    ])
