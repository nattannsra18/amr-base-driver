import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('amr_base_driver')
    enable_motors = LaunchConfiguration('enable_motors')
    enable_base_control = LaunchConfiguration('enable_base_control')
    enable_support = LaunchConfiguration('enable_support')
    enable_black_box = LaunchConfiguration('enable_black_box')
    publish_sensor_tf = LaunchConfiguration('publish_sensor_tf')
    sensor_tf_enabled = PythonExpression([
        "'", enable_support, "' == 'true' and '",
        publish_sensor_tf, "' == 'true'",
    ])
    ekf_transform_time_offset = LaunchConfiguration(
        'ekf_transform_time_offset')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_motors', default_value='false',
            description='Allow cmd_vel to drive motors. Keep false until floor test.'),
        DeclareLaunchArgument(
            'enable_base_control', default_value='true',
            description='Start command arbitration and the ESP32 serial bridge.'),
        DeclareLaunchArgument(
            'enable_support', default_value='true',
            description='Start EKF, conditioned IMU, recovery, and static TF nodes.'),
        DeclareLaunchArgument(
            'enable_black_box', default_value='true',
            description=(
                'Run the rolling incident recorder in this launch. The boot '
                'deployment runs it in a separate low-priority service.')),
        DeclareLaunchArgument(
            'publish_sensor_tf', default_value='true',
            description='Publish measured base, IMU, and LiDAR static transforms.'),
        DeclareLaunchArgument(
            'ekf_transform_time_offset', default_value='0.0',
            description=(
                'Seconds to future-date odom to base TF. Mapping keeps zero; '
                'localization may override this to cover measured TF latency.')),
        Node(
            package='amr_base_driver',
            executable='cmd_vel_passthrough',
            name='cmd_vel_passthrough',
            output='screen',
            condition=IfCondition(enable_base_control),
        ),
        Node(
            package='amr_base_driver',
            executable='guarded_reverse',
            name='guarded_reverse',
            output='screen',
            parameters=[{'scan_topic': '/scan'}],
            condition=IfCondition(enable_support),
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
            condition=IfCondition(enable_base_control),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'ekf.yaml'),
                {'transform_time_offset': ParameterValue(
                    ekf_transform_time_offset, value_type=float)},
            ],
            remappings=[('odometry/filtered', '/odometry/filtered')],
            condition=IfCondition(enable_support),
        ),
        Node(
            package='amr_base_driver',
            executable='imu_conditioner',
            name='imu_conditioner',
            output='screen',
            condition=IfCondition(enable_support),
        ),
        Node(
            package='amr_base_driver',
            executable='black_box_recorder',
            name='black_box_recorder',
            output='screen',
            # Incident capture must never preempt serial I/O or control work.
            prefix='nice -n 10',
            parameters=[{'max_topic_hz': 1.0}],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(enable_black_box),
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
            condition=IfCondition(sensor_tf_enabled),
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
            condition=IfCondition(sensor_tf_enabled),
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
            condition=IfCondition(sensor_tf_enabled),
        ),
    ])
