import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_share = get_package_share_directory('amr_base_driver')

    map_file = LaunchConfiguration('map_yaml')
    enable_motors = LaunchConfiguration('enable_motors')
    enable_drive_supervisor = LaunchConfiguration('enable_drive_supervisor')
    ekf_transform_time_offset = LaunchConfiguration(
        'ekf_transform_time_offset')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'base_hardware.launch.py')),
        launch_arguments={
            'enable_motors': enable_motors,
            'enable_drive_supervisor': enable_drive_supervisor,
            'publish_sensor_tf': 'true',
            'ekf_transform_time_offset': ekf_transform_time_offset,
        }.items(),
    )
    lidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(base_share, 'config', 'ydlidar_x3.yaml')],
        remappings=[('scan', '/scan_raw')],
    )
    scan_resampler = Node(
        package='amr_base_driver',
        executable='scan_resampler',
        name='scan_resampler',
        output='screen',
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan',
            'output_beams': 360,
        }],
    )
    localization_params = os.path.join(
        base_share, 'config', 'localization.yaml')
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[localization_params, {'yaml_filename': map_file}],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[localization_params],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    default_map = os.path.join(
        os.path.expanduser('~'), 'amr_ws', 'maps',
        'slam_post_wheel_repair_20260920_082948.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml', default_value=default_map,
            description='Absolute path to the occupancy-grid map YAML.'),
        DeclareLaunchArgument(
            'enable_motors', default_value='false',
            description='Unlock motion only for an attended localization test.'),
        DeclareLaunchArgument(
            'enable_drive_supervisor', default_value='false',
            description=(
                'DEPRECATED/EXPERIMENTAL. Keep disabled for the supported '
                'localization and navigation path.')),
        DeclareLaunchArgument(
            'ekf_transform_time_offset', default_value='0.10',
            description=(
                'Future offset for odom to base TF during localization. '
                'The measured controller-side lag reached 93 ms.')),
        base_launch,
        TimerAction(period=3.0, actions=[lidar, scan_resampler]),
        TimerAction(
            period=8.0,
            actions=[map_server, amcl, lifecycle_manager]),
    ])
