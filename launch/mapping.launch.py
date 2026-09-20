import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_share = get_package_share_directory('amr_base_driver')
    slam_share = get_package_share_directory('slam_toolbox')
    enable_motors = LaunchConfiguration('enable_motors')
    enable_drive_supervisor = LaunchConfiguration('enable_drive_supervisor')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'base_hardware.launch.py')),
        launch_arguments={
            'enable_motors': enable_motors,
            'enable_drive_supervisor': enable_drive_supervisor,
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
        DeclareLaunchArgument(
            'enable_drive_supervisor', default_value='false',
            description=(
                'DEPRECATED/EXPERIMENTAL caster supervisor. Mapping uses '
                'cmd_vel_passthrough by default.')),
        base_launch,
        # Bring up the base TF/odometry chain before scans, then start SLAM
        # only after the laser has begun publishing.  This prevents the
        # one-time message-filter queue overflow seen during simultaneous
        # startup on the ODROID-C4.
        TimerAction(period=3.0, actions=[lidar, scan_resampler]),
        # Leave a full TF/odometry history window before slam_toolbox starts.
        # Starting at six seconds still raced the first resampled scan against
        # the newest odom transform on the C4 and discarded one startup frame.
        TimerAction(period=8.0, actions=[slam]),
    ])
