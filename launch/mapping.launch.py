import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_share = get_package_share_directory('amr_base_driver')
    slam_share = get_package_share_directory('slam_toolbox')
    enable_motors = LaunchConfiguration('enable_motors')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'base_hardware.launch.py')),
        launch_arguments={
            'enable_motors': enable_motors,
            'publish_sensor_tf': 'true',
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
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_sync_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'slam_params_file': os.path.join(
                base_share, 'config', 'slam_mapping.yaml'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_motors', default_value='false',
            description='Unlock cmd_vel only during an attended floor test.'),
        base_launch,
        # Subscribers can start before their publishers.  Avoid fixed sleeps:
        # slam_toolbox will begin updating when scan and TF are both ready.
        lidar,
        scan_resampler,
        slam,
    ])
