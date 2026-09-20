"""Correct residual MPU6050 bias only during confirmed stationary intervals."""
import copy
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .imu_bias import StationaryBias


class ImuConditioner(Node):
    def __init__(self):
        super().__init__('imu_conditioner')
        self.estimator = StationaryBias()
        self.last_odom = -100.0
        self.last_motion = -100.0
        self.last_cmd = -100.0
        self.command_moving = False
        self.samples = 0
        self.corrected = self.create_publisher(Imu, '/imu/data', 20)
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 20)
        self.create_subscription(Odometry, '/wheel/odometry', self.on_odom, 20)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0, self.diagnostics)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd(self, msg):
        self.last_cmd = self.now()
        self.command_moving = abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001

    def on_odom(self, msg):
        self.last_odom = self.now()
        if (abs(msg.twist.twist.linear.x) > 0.001 or
                abs(msg.twist.twist.angular.z) > 0.001):
            self.last_motion = self.last_odom

    def on_imu(self, msg):
        now = self.now()
        commanded = self.command_moving and now-self.last_cmd < 0.5
        stationary = (0 <= now-self.last_odom < 0.2 and
                      now-self.last_motion > 2.0 and not commanded)
        g = msg.angular_velocity
        a = msg.linear_acceleration
        bias = self.estimator.update(
            now, [g.x, g.y, g.z],
            math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z), stationary)
        out = copy.deepcopy(msg)
        out.angular_velocity.x -= bias[0]
        out.angular_velocity.y -= bias[1]
        out.angular_velocity.z -= bias[2]
        variance = 2.5e-5 if self.estimator.ready else 4.0e-4
        out.angular_velocity_covariance = [
            variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
        self.corrected.publish(out)
        self.samples += 1

    def diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'MPU6050 residual bias'
        status.hardware_id = 'mpu6050'
        status.level = DiagnosticStatus.OK if self.estimator.ready else DiagnosticStatus.WARN
        status.message = ('Stationary bias calibrated' if self.estimator.ready else
                          'Waiting for five seconds of stable stationary data')
        status.values = [KeyValue(key='samples', value=str(self.samples))]
        for axis, bias in zip('xyz', self.estimator.bias):
            status.values.append(KeyValue(key=f'gyro_{axis}_bias_dps',
                                          value=f'{math.degrees(bias):.5f}'))
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diag.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuConditioner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
