"""ROS 2 drive command supervisor for the passive-caster robot base."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .drive_supervisor_core import DriveSupervisorCore


def quaternion_yaw(quaternion):
    sin_yaw = 2.0 * (quaternion.w * quaternion.z +
                     quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y +
                           quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class DriveSupervisor(Node):
    def __init__(self):
        super().__init__('drive_supervisor')
        defaults = {
            'input_timeout': 0.25,
            'control_rate': 20.0,
            'linear_accel': 0.20,
            'linear_decel': 0.45,
            'angular_accel': 0.8,
            'align_speed': 0.10,
            'align_distance': 0.12,
            'align_timeout': 2.2,
            'stop_timeout': 0.35,
            'heading_kp': 0.8,
            'heading_max': 0.12,
            'heading_min': 0.03,
            'align_heading_max': 0.05,
            'heading_tolerance_deg': 1.5,
            'initial_alignment': True,
            'straight_heading_kp': 0.7,
            'straight_heading_kd': 0.15,
            'straight_heading_max': 0.10,
            'straight_heading_tolerance_deg': 1.2,
            'straight_angular_deadband': 0.01,
            'heading_hold_ramp_time': 1.0,
            'yaw_rate_filter_tau': 0.25,
            'odom_timeout': 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        value = lambda name: self.get_parameter(name).value
        self.input_timeout = float(value('input_timeout'))
        self.odom_timeout = float(value('odom_timeout'))
        self.rate = float(value('control_rate'))
        self.core = DriveSupervisorCore(
            linear_accel=float(value('linear_accel')),
            linear_decel=float(value('linear_decel')),
            angular_accel=float(value('angular_accel')),
            align_speed=float(value('align_speed')),
            align_distance=float(value('align_distance')),
            align_timeout=float(value('align_timeout')),
            stop_timeout=float(value('stop_timeout')),
            heading_kp=float(value('heading_kp')),
            heading_max=float(value('heading_max')),
            heading_min=float(value('heading_min')),
            align_heading_max=float(value('align_heading_max')),
            heading_tolerance=math.radians(
                float(value('heading_tolerance_deg'))),
            initial_alignment=bool(value('initial_alignment')),
            straight_heading_kp=float(value('straight_heading_kp')),
            straight_heading_kd=float(value('straight_heading_kd')),
            straight_heading_max=float(value('straight_heading_max')),
            straight_heading_tolerance=math.radians(
                float(value('straight_heading_tolerance_deg'))),
            straight_angular_deadband=float(
                value('straight_angular_deadband')),
            heading_hold_ramp_time=float(value('heading_hold_ramp_time')),
            yaw_rate_filter_tau=float(value('yaw_rate_filter_tau')),
        )

        self.target = Twist()
        self.last_input = 0.0
        self.last_odom = 0.0
        self.x = self.y = self.yaw = None
        self.measured_linear = None
        self.measured_angular = None
        self.previous_tick = time.monotonic()
        self.last_reported_state = None

        self.output_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.state_pub = self.create_publisher(
            String, '/drive_supervisor/state', 10)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 20)
        self.create_timer(1.0 / max(self.rate, 1.0), self.control_tick)
        self.create_timer(1.0, self.diagnostic_tick)
        self.get_logger().info(
            'Drive supervisor: /cmd_vel -> /cmd_vel_safe; '
            'passive-caster reversal handling enabled')

    def command_callback(self, message):
        self.target = message
        self.last_input = time.monotonic()

    def odom_callback(self, message):
        self.x = float(message.pose.pose.position.x)
        self.y = float(message.pose.pose.position.y)
        self.yaw = quaternion_yaw(message.pose.pose.orientation)
        self.measured_linear = float(message.twist.twist.linear.x)
        self.measured_angular = float(message.twist.twist.angular.z)
        self.last_odom = time.monotonic()

    def control_tick(self):
        now = time.monotonic()
        dt = now - self.previous_tick
        self.previous_tick = now
        input_fresh = bool(
            self.last_input and now - self.last_input <= self.input_timeout)
        odom_fresh = bool(
            self.last_odom and now - self.last_odom <= self.odom_timeout)
        # Heading-aware caster handling must fail closed if EKF odometry is
        # unavailable.  Explicit zero still publishes zero normally.
        motion_requested = (
            abs(float(self.target.linear.x)) >= 0.005 or
            abs(float(self.target.angular.z)) >= 0.005)
        safe_input = input_fresh and (odom_fresh or not motion_requested)
        linear, angular, state = self.core.update(
            now, dt,
            float(self.target.linear.x), float(self.target.angular.z),
            input_fresh=safe_input,
            x=self.x if odom_fresh else None,
            y=self.y if odom_fresh else None,
            yaw=self.yaw if odom_fresh else None,
            measured_linear=self.measured_linear if odom_fresh else None,
            measured_angular=self.measured_angular if odom_fresh else None,
        )
        output = Twist()
        output.linear.x = linear
        output.angular.z = angular
        self.output_pub.publish(output)
        if state != self.last_reported_state:
            self.last_reported_state = state
            self.state_pub.publish(String(data=state))
            yaw_text = ('unavailable' if self.yaw is None else
                        f'{math.degrees(self.yaw):.2f} deg')
            self.get_logger().info(
                f'Drive state: {state}; yaw={yaw_text}; '
                f'heading_error={math.degrees(self.core.heading_error):.2f} deg')

    def diagnostic_tick(self):
        now = time.monotonic()
        input_age = now - self.last_input if self.last_input else float('inf')
        odom_age = now - self.last_odom if self.last_odom else float('inf')
        status = DiagnosticStatus()
        status.name = 'Drive supervisor'
        status.hardware_id = 'odroid-c4'
        motion_requested = (
            abs(float(self.target.linear.x)) >= 0.005 or
            abs(float(self.target.angular.z)) >= 0.005)
        if odom_age > self.odom_timeout and motion_requested:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Odometry stale; motion inhibited'
        elif odom_age > self.odom_timeout:
            status.level = DiagnosticStatus.WARN
            status.message = 'Odometry stale; heading correction unavailable'
        else:
            status.level = DiagnosticStatus.OK
            status.message = self.core.state
        status.values = [
            KeyValue(key='state', value=self.core.state),
            KeyValue(key='input_age_s', value=f'{input_age:.3f}'),
            KeyValue(key='odom_age_s', value=f'{odom_age:.3f}'),
            KeyValue(key='heading_error_deg', value=f'{math.degrees(self.core.heading_error):.2f}'),
            KeyValue(key='output_linear', value=f'{self.core.output_linear:.3f}'),
            KeyValue(key='output_angular', value=f'{self.core.output_angular:.3f}'),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diag_pub.publish(message)

    def destroy_node(self):
        # A launch-wide SIGINT may invalidate the ROS context before this
        # method runs.  The firmware watchdog still stops the base, and this
        # guard avoids turning a clean shutdown into RCLError noise.
        if rclpy.ok():
            stop = Twist()
            for _ in range(4):
                self.output_pub.publish(stop)
                time.sleep(0.02)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
