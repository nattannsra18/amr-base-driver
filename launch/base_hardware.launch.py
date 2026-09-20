import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('amr_base_driver')
    enable_motors = LaunchConfiguration('enable_motors')
    enable_drive_supervisor = LaunchConfiguration('enable_drive_supervisor')
    publish_sensor_tf = LaunchConfiguration('publish_sensor_tf')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_motors', default_value='false',
            description='Allow cmd_vel to drive motors. Keep false until floor test.'),
        DeclareLaunchArgument(
            'enable_drive_supervisor', default_value='false',
            description=(
                'DEPRECATED/EXPERIMENTAL caster supervisor. The supported '
                'default path uses cmd_vel_passthrough. Enable only for an '
                'explicit, attended regression test.')),
        DeclareLaunchArgument(
            'publish_sensor_tf', default_value='true',
            description='Publish measured base, IMU, and LiDAR static transforms.'),
        Node(
            package='amr_base_driver',
            executable='drive_supervisor',
            name='drive_supervisor',
            output='screen',
            parameters=[os.path.join(
                share, 'config', 'drive_supervisor.yaml')],
            condition=IfCondition(enable_drive_supervisor),
        ),
        Node(
            package='amr_base_driver',
            executable='cmd_vel_passthrough',
            name='cmd_vel_passthrough',
            output='screen',
            condition=UnlessCondition(enable_drive_supervisor),
        ),
        Node(
            package='amr_base_driver',
            executable='serial_bridge',
            name='amr_base_serial_bridge',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'base_driver.yaml'),
                {'enable_motors': enable_motors},
            ],
            remappings=[('cmd_vel', '/cmd_vel_safe')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(share, 'config', 'ekf.yaml')],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
        Node(
            package='amr_base_driver',
            executable='imu_conditioner',
            name='imu_conditioner',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_static_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.0325',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link',
            ],
            condition=IfCondition(publish_sensor_tf),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=[
                '--x', '0.273', '--y', '0', '--z', '0.0425',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
            ],
            condition=IfCondition(publish_sensor_tf),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=[
                '--x', '-0.042', '--y', '-0.005', '--z', '0.3625',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
            ],
            condition=IfCondition(publish_sensor_tf),
        ),
    ])
