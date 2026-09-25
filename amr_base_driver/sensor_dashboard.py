import math
import os
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_srvs.srv import Empty


def age_text(timestamp):
    if not timestamp:
        return 'never'
    return f'{time.monotonic() - timestamp:4.1f}s'


def quaternion_yaw(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class SensorDashboard(Node):
    def __init__(self):
        super().__init__('amr_sensor_dashboard')
        self.declare_parameter('left_cpr', 2362.4)
        self.declare_parameter('right_cpr', 2367.0)
        self.left_cpr = float(self.get_parameter('left_cpr').value)
        self.right_cpr = float(self.get_parameter('right_cpr').value)
        self.joint = None
        self.imu = None
        self.odom = None
        self.scan = None
        self.diagnostic = None
        self.diagnostic_fields = {}
        self.times = {}
        self.scan_rate = 0.0
        self.previous_scan_time = 0.0
        self.travelled = 0.0
        self.previous_xy = None
        self.create_subscription(
            JointState, '/joint_states', self.on_joint, 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 20)
        self.create_subscription(
            Odometry, '/wheel/odometry', self.on_odom, 20)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self.on_diagnostic, 10)
        self.create_service(
            Empty, '/reset_dashboard_distance', self.on_reset_distance)
        self.create_timer(0.2, self.render)

    def on_reset_distance(self, _request, response):
        self.travelled = 0.0
        self.previous_xy = None
        return response

    def mark(self, name):
        self.times[name] = time.monotonic()

    def on_joint(self, message):
        self.joint = message
        self.mark('joint')

    def on_imu(self, message):
        self.imu = message
        self.mark('imu')

    def on_odom(self, message):
        self.odom = message
        xy = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self.previous_xy is not None:
            step = math.hypot(
                xy[0] - self.previous_xy[0], xy[1] - self.previous_xy[1])
            if step < 0.5:
                self.travelled += step
        self.previous_xy = xy
        self.mark('odom')

    def on_scan(self, message):
        self.scan = message
        now = time.monotonic()
        if self.previous_scan_time:
            instant_rate = 1.0 / max(now - self.previous_scan_time, 1e-6)
            self.scan_rate = (
                instant_rate if not self.scan_rate
                else 0.8 * self.scan_rate + 0.2 * instant_rate)
        self.previous_scan_time = now
        self.mark('scan')

    def on_diagnostic(self, message):
        for status in message.status:
            if status.name in {
                'ESP32 base controller',
                'ESP32 MCU fault',
                'Base drive telemetry',
                'Base serial telemetry',
            }:
                self.diagnostic = status
                self.diagnostic_fields.update(
                    {item.key: item.value for item in status.values})
                self.mark('diagnostic')

    def joint_values(self):
        if self.joint is None or len(self.joint.position) < 2:
            return (0, 0, 0.0, 0.0)
        counts = (
            round(self.joint.position[0] * self.left_cpr / (2.0 * math.pi)),
            round(self.joint.position[1] * self.right_cpr / (2.0 * math.pi)))
        velocity = list(self.joint.velocity) + [0.0, 0.0]
        rpm_scale = 60.0 / (2.0 * math.pi)
        return (counts[0], counts[1],
                velocity[0] * rpm_scale, velocity[1] * rpm_scale)

    def diagnostic_values(self):
        if self.diagnostic is None:
            return 'NO DATA', '-', '-', '-'
        values = self.diagnostic_fields
        return (self.diagnostic.message,
                values.get('mcu_fault', '-'),
                values.get('mcu_flags', '-'),
                values.get('motors_enabled', '-'))

    def render(self):
        left_count, right_count, left_rpm, right_rpm = self.joint_values()
        status, fault, flags, motors = self.diagnostic_values()
        if self.odom:
            pose = self.odom.pose.pose
            x, y = pose.position.x, pose.position.y
            yaw = math.degrees(quaternion_yaw(pose.orientation))
            linear = self.odom.twist.twist.linear.x
            angular = self.odom.twist.twist.angular.z
        else:
            x = y = yaw = linear = angular = 0.0
        if self.imu:
            accel = self.imu.linear_acceleration
            gyro = self.imu.angular_velocity
            gravity = 9.80665
            ax, ay, az = accel.x / gravity, accel.y / gravity, accel.z / gravity
            gz = math.degrees(gyro.z)
        else:
            ax = ay = az = gz = 0.0
        valid_ranges = [] if self.scan is None else [
            value for value in self.scan.ranges
            if math.isfinite(value) and value > 0.0]
        scan_points = len(valid_ranges)
        nearest = min(valid_ranges) if valid_ranges else 0.0
        print('\033[2J\033[H', end='')
        print('AMR REAL-TIME SENSOR DASHBOARD  (Ctrl-C to close)')
        print('=' * 62)
        print(f'ESP32 : {status}')
        print(f'Fault : {fault:<4} Flags: {flags:<5} Motors enabled: {motors}')
        print('-' * 62)
        print('Wheel       Encoder count       RPM')
        print(f'Left        {left_count:>13d}     {left_rpm:>8.2f}')
        print(f'Right       {right_count:>13d}     {right_rpm:>8.2f}')
        print('-' * 62)
        print(f'Odometry    x={x:+.3f} m  y={y:+.3f} m  yaw={yaw:+.1f} deg')
        print(f'Velocity    linear={linear:+.3f} m/s  angular={angular:+.3f} rad/s')
        print(f'Total path  {self.travelled:.3f} m (accumulated wheel odometry)')
        print('-' * 62)
        print(f'IMU         ax={ax:+.3f} g  ay={ay:+.3f} g  az={az:+.3f} g')
        print(f'            gyro-z={gz:+.2f} deg/s')
        print(f'LiDAR       {self.scan_rate:5.1f} Hz  valid={scan_points:3d}  '
              f'nearest={nearest:.3f} m')
        print('-' * 62)
        print('Freshness   encoder={} imu={} odom={} scan={} diagnostics={}'.format(
            age_text(self.times.get('joint')),
            age_text(self.times.get('imu')),
            age_text(self.times.get('odom')),
            age_text(self.times.get('scan')),
            age_text(self.times.get('diagnostic'))))
        print('Distance is provisional until loaded wheel-radius calibration.')
        print(f'Host: {os.uname().nodename}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = SensorDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
