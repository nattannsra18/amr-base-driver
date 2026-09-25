import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Empty


def quaternion_yaw(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def wrapped_angle_delta(current, previous):
    return math.atan2(
        math.sin(current - previous), math.cos(current - previous))


class CalibrationRun(Node):
    def __init__(self):
        super().__init__('amr_calibration_run')
        self.declare_parameter('mode', 'straight')
        # Calibration is a guarded sign check, not a long autonomous drive.
        # Keep defaults deliberately short so a misspelled ROS parameter
        # cannot turn a bench/HIL check into metres of unexpected motion.
        self.declare_parameter('target_distance', 0.12)
        self.declare_parameter('target_angle', 0.20)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('arc_linear_speed', 0.05)
        self.declare_parameter('arc_angular_speed', 0.28)
        self.declare_parameter('max_duration', 8.0)
        self.declare_parameter('distance_stop_margin', 0.0)
        self.declare_parameter('angle_stop_margin', 0.0)
        self.mode = str(self.get_parameter('mode').value)
        self.target_distance = float(
            self.get_parameter('target_distance').value)
        self.target_angle = float(self.get_parameter('target_angle').value)
        self.linear_speed = float(
            self.get_parameter('linear_speed').value)
        self.angular_speed = float(
            self.get_parameter('angular_speed').value)
        self.arc_linear_speed = float(
            self.get_parameter('arc_linear_speed').value)
        self.arc_angular_speed = float(
            self.get_parameter('arc_angular_speed').value)
        self.max_duration = float(
            self.get_parameter('max_duration').value)
        self.distance_stop_margin = float(
            self.get_parameter('distance_stop_margin').value)
        self.angle_stop_margin = float(
            self.get_parameter('angle_stop_margin').value)

        self.command_pub = self.create_publisher(
            Twist, '/cmd_vel_manual', 10)
        self.create_subscription(
            Odometry, '/wheel/odometry', self.on_odom, 20)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 20)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 20)
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self.on_diagnostic, 10)
        self.reset_client = self.create_client(
            Empty, '/reset_wheel_odometry')

        self.odom = None
        self.imu = None
        self.imu_angle = 0.0
        self.imu_collecting = False
        self.last_imu_time = None
        self.fault = None
        self.joint_positions = None
        self.motors_enabled = False
        self.diagnostic_time = 0.0

    def on_odom(self, message):
        self.odom = message

    def on_imu(self, message):
        now = time.monotonic()
        if self.imu_collecting and self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if 0.0 < dt < 0.20:
                self.imu_angle += message.angular_velocity.z * dt
        self.last_imu_time = now
        self.imu = message

    def on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        try:
            self.joint_positions = (
                float(positions['left_wheel_joint']),
                float(positions['right_wheel_joint']),
            )
        except (KeyError, TypeError, ValueError):
            return

    def on_diagnostic(self, message):
        for status in message.status:
            if status.name not in {
                'ESP32 base controller',
                'ESP32 MCU fault',
                'Base drive telemetry',
            }:
                continue
            values = {item.key: item.value for item in status.values}
            if 'mcu_fault' in values:
                try:
                    self.fault = int(values['mcu_fault'])
                except ValueError:
                    self.fault = -1
            if 'motors_enabled' in values:
                self.motors_enabled = values['motors_enabled'] == 'True'
            self.diagnostic_time = time.monotonic()

    def publish_command(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.command_pub.publish(message)

    def stop(self):
        for _ in range(12):
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def wait_until_ready(self, timeout=45.0):
        deadline = time.monotonic() + timeout
        last_report = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            diagnostic_age = (
                now - self.diagnostic_time
                if self.diagnostic_time else float('inf'))
            diagnostic_fresh = diagnostic_age < 2.0
            if (self.odom is not None and self.joint_positions is not None
                    and diagnostic_fresh and
                    self.fault == 0 and self.motors_enabled):
                return True
            if now - last_report >= 1.0:
                print(
                    'READY_WAIT '
                    f'odom={self.odom is not None} '
                    f'joints={self.joint_positions is not None} '
                    f'diagnostic_age={diagnostic_age:.2f}s '
                    f'mcu_fault={self.fault} '
                    f'motors_enabled={self.motors_enabled}',
                    flush=True)
                last_report = now
        return False

    def reset_odometry(self):
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self.reset_client.call_async(Empty.Request())
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done():
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > deadline:
                return False
        return future.done() and future.exception() is None

    def safety_ok(self):
        diagnostic_fresh = time.monotonic() - self.diagnostic_time < 2.0
        return diagnostic_fresh and self.fault == 0 and self.motors_enabled

    def run(self):
        if self.mode not in (
            'straight', 'reverse', 'turn_left', 'turn_right',
            'arc_left', 'arc_right',
        ):
            print(f'ABORT invalid mode: {self.mode}', flush=True)
            return 2
        print('Waiting for fresh odometry and healthy ESP32 diagnostics...',
              flush=True)
        if not self.wait_until_ready():
            print('ABORT system did not become ready', flush=True)
            self.stop()
            return 3
        if not self.reset_odometry():
            print('ABORT odometry reset failed', flush=True)
            self.stop()
            return 4

        for remaining in (3, 2, 1):
            print(f'Starting in {remaining}...', flush=True)
            end = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < end:
                self.publish_command()
                rclpy.spin_once(self, timeout_sec=0.05)

        start = time.monotonic()
        last_report = start
        last_xy = None
        last_yaw = None
        distance = 0.0
        wheel_angle = 0.0
        self.imu_angle = 0.0
        self.last_imu_time = start
        self.imu_collecting = True
        start_joints = self.joint_positions
        reason = 'timeout'

        while rclpy.ok() and time.monotonic() - start < self.max_duration:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not self.safety_ok():
                reason = f'safety abort fault={self.fault}'
                break
            if self.odom is None:
                continue

            position = self.odom.pose.pose.position
            current_xy = (position.x, position.y)
            current_yaw = quaternion_yaw(self.odom.pose.pose.orientation)
            if last_xy is not None:
                step = math.hypot(
                    current_xy[0] - last_xy[0],
                    current_xy[1] - last_xy[1])
                if step < 0.10:
                    distance += step
            if last_yaw is not None:
                wheel_angle += wrapped_angle_delta(current_yaw, last_yaw)
            last_xy = current_xy
            last_yaw = current_yaw

            now = time.monotonic()
            if self.mode in {'straight', 'reverse'}:
                direction = 1.0 if self.mode == 'straight' else -1.0
                self.publish_command(linear=direction * abs(self.linear_speed))
                if distance >= max(
                        self.target_distance - self.distance_stop_margin, 0.0):
                    reason = 'target reached'
                    break
            elif self.mode in {'arc_left', 'arc_right'}:
                direction = 1.0 if self.mode == 'arc_left' else -1.0
                self.publish_command(
                    linear=-abs(self.arc_linear_speed),
                    angular=direction * abs(self.arc_angular_speed),
                )
                if abs(wheel_angle) >= max(
                        self.target_angle - self.angle_stop_margin, 0.0):
                    reason = 'target reached'
                    break
            else:
                direction = 1.0 if self.mode == 'turn_left' else -1.0
                self.publish_command(
                    angular=direction * self.angular_speed)
                if abs(wheel_angle) >= max(
                        self.target_angle - self.angle_stop_margin, 0.0):
                    reason = 'target reached'
                    break

            if now - last_report >= 0.5:
                print(
                    f't={now - start:.1f}s distance={distance:.3f}m '
                    f'wheel_angle={math.degrees(wheel_angle):+.1f}deg '
                    f'imu_angle={math.degrees(self.imu_angle):+.1f}deg',
                    flush=True)
                last_report = now
            time.sleep(0.02)

        self.stop()
        self.imu_collecting = False
        coast_end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < coast_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        left_delta = self.joint_positions[0] - start_joints[0]
        right_delta = self.joint_positions[1] - start_joints[1]
        signs_ok = self.wheel_signs_match(left_delta, right_delta)
        print(
            f'RESULT mode={self.mode} reason={reason} '
            f'distance={distance:.4f}m '
            f'wheel_angle={math.degrees(wheel_angle):+.2f}deg '
            f'imu_angle={math.degrees(self.imu_angle):+.2f}deg '
            f'left_delta={left_delta:+.4f}rad '
            f'right_delta={right_delta:+.4f}rad signs_ok={signs_ok} '
            f'elapsed={time.monotonic() - start:.2f}s fault={self.fault}',
            flush=True)
        if reason != 'target reached':
            return 5
        return 0 if signs_ok else 6

    def wheel_signs_match(self, left_delta, right_delta):
        """Verify encoder direction before any stall tolerance is changed."""
        threshold = 0.05
        if self.mode == 'straight':
            return left_delta > threshold and right_delta > threshold
        if self.mode == 'reverse':
            return left_delta < -threshold and right_delta < -threshold
        if self.mode == 'arc_left':
            return (
                left_delta < -threshold
                and left_delta < right_delta
            )
        if self.mode == 'arc_right':
            return (
                right_delta < -threshold
                and right_delta < left_delta
            )
        if self.mode == 'turn_left':
            return left_delta < -threshold and right_delta > threshold
        return left_delta > threshold and right_delta < -threshold


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationRun()
    try:
        result = node.run()
    except KeyboardInterrupt:
        node.stop()
        result = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
