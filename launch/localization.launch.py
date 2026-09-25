import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_share = get_package_share_directory('amr_base_driver')

    map_file = LaunchConfiguration('map_yaml')
    enable_motors = LaunchConfiguration('enable_motors')
    start_localization = LaunchConfiguration('start_localization')
    ekf_transform_time_offset = LaunchConfiguration(
        'ekf_transform_time_offset')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'base_hardware.launch.py')),
        launch_arguments={
            'enable_motors': enable_motors,
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
            'telemetry_hz': 2.0,
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
        condition=IfCondition(start_localization),
        parameters=[localization_params, {'yaml_filename': map_file}],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=IfCondition(start_localization),
        parameters=[localization_params],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=IfCondition(start_localization),
        parameters=[{
            # The readiness gate owns startup and retries transient DDS load.
            'autostart': False,
            # DDS discovery on the C4 is briefly CPU-bound during bring-up.
            # Keep lifecycle readiness authoritative without a fixed sleep.
            'bond_timeout': 10.0,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    readiness_gate = Node(
        package='amr_base_driver',
        executable='localization_readiness_gate',
        name='localization_readiness_gate',
        output='screen',
        condition=IfCondition(start_localization),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml', default_value='',
            description='Absolute path to the selected occupancy-grid map YAML.'),
        DeclareLaunchArgument(
            'start_localization', default_value='true',
            description=(
                'Start map server and AMCL only when an active map has been '
                'selected. Base hardware and LiDAR remain available otherwise.')),
        DeclareLaunchArgument(
            'enable_motors', default_value='false',
            description='Unlock motion only for an attended localization test.'),
        DeclareLaunchArgument(
            'ekf_transform_time_offset', default_value='0.10',
            description=(
                'Future offset for odom to base TF during localization. '
                'The measured controller-side lag reached 93 ms.')),
        base_launch,
        # ROS subscriptions and lifecycle services are the readiness boundary;
        # fixed sleeps made startup slower and still raced on a loaded C4.
        lidar,
        scan_resampler,
        map_server,
        amcl,
        lifecycle_manager,
        readiness_gate,
    ])
