import binascii
import math
import os
import struct
import termios
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Empty, SetBool, Trigger

from .motion_guard import MotionGuard


TELEMETRY_MAGIC = b'\x5a\xa5'
TELEMETRY_FORMAT = '<HBBIIiihhhhhhhhhHBBH'
TELEMETRY_LENGTH = struct.calcsize(TELEMETRY_FORMAT)


def crc16_ccitt(data):
    return binascii.crc_hqx(data, 0xFFFF)


def wrapped_int32_delta(current, previous):
    delta = (current - previous) & 0xFFFFFFFF
    return delta - 0x100000000 if delta & 0x80000000 else delta


def yaw_quaternion(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class SerialBridge(Node):
    def __init__(self):
        super().__init__('amr_base_serial_bridge')
        self.declare_parameter(
            'port',
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-'
            'if00-port0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.032085)
        self.declare_parameter('wheel_separation', 0.36094)
        self.declare_parameter('left_cpr', 2362.4)
        self.declare_parameter('right_cpr', 2367.0)
        self.declare_parameter('accel_scale', 0.9372)
        self.declare_parameter('max_rpm', 90.0)
        self.declare_parameter('cmd_vel_timeout', 0.25)
        self.declare_parameter('telemetry_publish_divisor', 2)
        self.declare_parameter('enable_motors', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'imu_link')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.radius = float(self.get_parameter('wheel_radius').value)
        self.separation = float(self.get_parameter('wheel_separation').value)
        self.left_cpr = float(self.get_parameter('left_cpr').value)
        self.right_cpr = float(self.get_parameter('right_cpr').value)
        self.accel_scale = float(self.get_parameter('accel_scale').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.cmd_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.telemetry_publish_divisor = max(
            1,
            int(self.get_parameter('telemetry_publish_divisor').value),
        )
        self.motors_enabled = bool(self.get_parameter('enable_motors').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.serial_port = None
        self.rx_buffer = bytearray()
        self.last_open_attempt = 0.0
        self.last_telemetry_monotonic = 0.0
        self.last_cmd_monotonic = 0.0
        self.last_diag_monotonic = 0.0
        self.command_sequence = 0
        self.telemetry_count = 0
        self.parse_errors = 0
        self.mcu_flags = 0
        self.mcu_fault = 0
        self.temperature = float('nan')
        self.requested_linear = 0.0
        self.requested_angular = 0.0
        self.motion_guard = MotionGuard()

        self.previous_left = None
        self.previous_right = None
        self.previous_mcu_ms = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odometry', 20)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 20)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_service(Empty, 'reset_wheel_odometry', self.reset_odom)
        self.create_service(
            SetBool, 'set_motors_enabled', self.set_motors_enabled)
        self.create_service(
            Trigger, 'clear_motor_fault', self.clear_motor_fault)
        self.create_service(
            SetBool, 'finish_motor_recovery', self.finish_motor_recovery)
        self.create_timer(0.01, self.io_tick)
        self.create_timer(0.10, self.command_tick)
        # Pre-stall is deliberately detected before the ESP32's hard encoder
        # latch. Publish it at the same practical cadence as command updates
        # so the recovery coordinator can stop, inspect the scan, and select
        # one escape before that hardware deadline expires.
        self.create_timer(0.10, self.diagnostic_tick)

        self.get_logger().info(
            f'ESP32 bridge starting on {self.port}; '
            f'enable_motors={self.motors_enabled}')
        if not self.motors_enabled:
            self.get_logger().warn(
                'Motors are locked at zero for bench/ROS topic validation.')

    def reset_odom(self, _request, response):
        self.x = self.y = self.yaw = 0.0
        self.previous_left = self.previous_right = None
        self.previous_mcu_ms = None
        return response

    def clear_motor_fault(self, _request, response):
        """Clear a latched MCU/host fault only after forcing motor stop."""
        self.requested_linear = 0.0
        self.requested_angular = 0.0
        self.last_cmd_monotonic = 0.0
        self.write_line('STOP')
        if self.motion_guard.pre_stall:
            if not self.motion_guard.begin_recovery(time.monotonic()):
                response.success = False
                response.message = 'Pre-stall recovery is not available'
                return response
            response.success = True
            response.message = (
                'Pre-stall recovery authorized once; MCU fault was not cleared')
            return response

        self.motion_guard = MotionGuard()
        self.write_line('CLEAR')
        response.success = True
        response.message = (
            'STOP and CLEAR sent; verify diagnostics mcu_fault=0 before motion')
        return response

    def finish_motor_recovery(self, request, response):
        """Commit the bounded pre-stall maneuver result."""
        if not self.motion_guard.finish_recovery(bool(request.data)):
            response.success = False
            response.message = 'No pre-stall recovery is in progress'
            return response
        self.write_line('STOP')
        response.success = True
        response.message = (
            'Recovery completed; wheel feedback must remain healthy to rearm'
            if request.data else
            f'Recovery failed; latched {self.motion_guard.fault}'
        )
        return response

    def set_motors_enabled(self, request, response):
        """Arm or lock motor output without restarting localization/Nav2."""
        self.requested_linear = 0.0
        self.requested_angular = 0.0
        self.last_cmd_monotonic = 0.0
        self.write_line('STOP')

        if not request.data:
            self.motors_enabled = False
            response.success = True
            response.message = 'Motors locked; STOP sent'
            self.get_logger().warning(response.message)
            return response

        now = time.monotonic()
        telemetry_age = (
            now - self.last_telemetry_monotonic
            if self.last_telemetry_monotonic else float('inf'))
        blockers = []
        if self.serial_port is None:
            blockers.append('serial disconnected')
        if telemetry_age > 0.3:
            blockers.append(f'telemetry stale ({telemetry_age:.3f}s)')
        if self.mcu_fault:
            blockers.append(f'mcu_fault={self.mcu_fault}')
        if self.motion_guard.blocks_motion:
            blockers.append(f'host_fault={self.motion_guard.fault}')
        if blockers:
            self.motors_enabled = False
            response.success = False
            response.message = 'Arm rejected: ' + ', '.join(blockers)
            self.get_logger().error(response.message)
            return response

        self.motors_enabled = True
        response.success = True
        response.message = (
            'Motors armed; waiting for a new, fresh cmd_vel command')
        self.get_logger().warning(response.message)
        return response

    def cmd_callback(self, message):
        self.requested_linear = float(message.linear.x)
        self.requested_angular = float(message.angular.z)
        self.last_cmd_monotonic = time.monotonic()

    def open_serial(self):
        now = time.monotonic()
        if now - self.last_open_attempt < 2.0:
            return
        self.last_open_attempt = now
        try:
            # Use a raw POSIX descriptor so the bridge also works with the
            # ODROID header UART and never changes USB modem-control lines.
            descriptor = os.open(
                self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attributes = termios.tcgetattr(descriptor)
            attributes[0] = 0
            attributes[1] = 0
            attributes[2] = (
                attributes[2]
                & ~(termios.CSIZE | termios.PARENB |
                    termios.CSTOPB | termios.HUPCL)
                | termios.CS8 | termios.CLOCAL | termios.CREAD)
            attributes[3] = 0
            attributes[4] = termios.B115200
            attributes[5] = termios.B115200
            termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
            termios.tcflush(descriptor, termios.TCIFLUSH)
            self.serial_port = descriptor
            self.rx_buffer.clear()
            self.get_logger().info('ESP32 serial port connected')
        except OSError as error:
            self.serial_port = None
            self.get_logger().warning(f'Waiting for ESP32 serial port: {error}')

    def close_serial(self, error):
        if self.serial_port is not None:
            try:
                os.close(self.serial_port)
            except OSError:
                pass
        self.serial_port = None
        self.get_logger().error(f'ESP32 serial disconnected: {error}')

    def io_tick(self):
        if self.serial_port is None:
            self.open_serial()
            return
        try:
            try:
                self.rx_buffer.extend(os.read(self.serial_port, 4096))
            except BlockingIOError:
                pass
            self.extract_telemetry_frames()
            if len(self.rx_buffer) > 4096:
                self.rx_buffer.clear()
                self.parse_errors += 1
        except OSError as error:
            self.close_serial(error)

    def write_line(self, line):
        if self.serial_port is None:
            return
        try:
            os.write(self.serial_port, (line + '\n').encode('ascii'))
        except (OSError, BlockingIOError) as error:
            self.close_serial(error)

    def command_tick(self):
        left_rpm = right_rpm = 0.0
        now = time.monotonic()
        fresh = now - self.last_cmd_monotonic <= self.cmd_timeout
        healthy = (now - self.last_telemetry_monotonic < 0.3 and
                   self.mcu_fault == 0 and not self.motion_guard.blocks_motion)
        if not healthy:
            self.motion_guard.command(now, [0.0, 0.0])
            self.write_line('STOP')
            return
        if self.motors_enabled and fresh:
            left_speed = (self.requested_linear -
                          0.5 * self.separation * self.requested_angular)
            right_speed = (self.requested_linear +
                           0.5 * self.separation * self.requested_angular)
            scale = 60.0 / (2.0 * math.pi * self.radius)
            left_rpm = left_speed * scale
            right_rpm = right_speed * scale
            peak = max(abs(left_rpm), abs(right_rpm))
            if peak > self.max_rpm:
                ratio = self.max_rpm / peak
                left_rpm *= ratio
                right_rpm *= ratio
        self.motion_guard.command(now, [left_rpm, right_rpm])
        self.write_line(
            f'CMD,{self.command_sequence},{left_rpm:.3f},{right_rpm:.3f}')
        self.command_sequence = (self.command_sequence + 1) & 0xFFFFFFFF

    def extract_telemetry_frames(self):
        while len(self.rx_buffer) >= 4:
            start = self.rx_buffer.find(TELEMETRY_MAGIC)
            if start < 0:
                # Retain a possible first magic byte split across reads.
                self.rx_buffer[:] = self.rx_buffer[-1:] \
                    if self.rx_buffer[-1:] == TELEMETRY_MAGIC[:1] else b''
                return
            if start:
                del self.rx_buffer[:start]
            if len(self.rx_buffer) < 4:
                return
            length = self.rx_buffer[3]
            if length != TELEMETRY_LENGTH:
                del self.rx_buffer[0]
                self.parse_errors += 1
                continue
            if len(self.rx_buffer) < length:
                return
            frame = bytes(self.rx_buffer[:length])
            del self.rx_buffer[:length]
            if crc16_ccitt(frame[:-2]) != int.from_bytes(
                    frame[-2:], 'little'):
                self.parse_errors += 1
                continue
            self.handle_telemetry_frame(frame)

    def handle_telemetry_frame(self, frame):
        try:
            values = struct.unpack(TELEMETRY_FORMAT, frame)
        except struct.error:
            self.parse_errors += 1
            return
        if values[0] != 0xA55A or values[1] != 1:
            self.parse_errors += 1
            return
        mcu_ms = values[4]
        left_count, right_count = values[5:7]
        left_rpm, right_rpm = (value / 100.0 for value in values[7:9])
        accel = [value / 1000.0 for value in values[9:12]]
        gyro = [value / 100.0 for value in values[12:15]]
        self.temperature = values[15] / 100.0
        self.mcu_flags = values[16]
        self.mcu_fault = values[17]

        self.last_telemetry_monotonic = time.monotonic()
        previous_fault = self.motion_guard.fault
        previous_pre_stall = self.motion_guard.pre_stall
        reason = self.motion_guard.sample(
            self.last_telemetry_monotonic, [left_rpm, right_rpm])
        if reason and not previous_fault:
            self.write_line('STOP')
            self.get_logger().error(f'Motors stopped: {reason}')
        elif self.motion_guard.pre_stall and not previous_pre_stall:
            self.write_line('STOP')
            self.get_logger().warning(
                'Motors paused before fault latch: '
                f'{self.motion_guard.pre_stall}; waiting for one recovery attempt')
        self.telemetry_count += 1
        if self.telemetry_count % self.telemetry_publish_divisor:
            return
        stamp = self.get_clock().now().to_msg()
        self.publish_imu(stamp, accel, gyro)
        self.publish_joint_state(stamp, left_count, right_count,
                                 left_rpm, right_rpm)
        self.publish_odometry(stamp, mcu_ms, left_count, right_count)

    def publish_imu(self, stamp, accel_g, gyro_dps):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self.imu_frame
        message.orientation_covariance[0] = -1.0
        degree_to_radian = math.pi / 180.0
        message.angular_velocity.x = gyro_dps[0] * degree_to_radian
        message.angular_velocity.y = gyro_dps[1] * degree_to_radian
        message.angular_velocity.z = gyro_dps[2] * degree_to_radian
        gyro_variance = 2.5e-5
        message.angular_velocity_covariance = [
            gyro_variance, 0.0, 0.0,
            0.0, gyro_variance, 0.0,
            0.0, 0.0, gyro_variance]
        gravity = 9.80665 * self.accel_scale
        message.linear_acceleration.x = accel_g[0] * gravity
        message.linear_acceleration.y = accel_g[1] * gravity
        message.linear_acceleration.z = accel_g[2] * gravity
        message.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04]
        self.imu_pub.publish(message)

    def publish_joint_state(self, stamp, left_count, right_count,
                            left_rpm, right_rpm):
        message = JointState()
        message.header.stamp = stamp
        message.name = ['left_wheel_joint', 'right_wheel_joint']
        message.position = [
            left_count * 2.0 * math.pi / self.left_cpr,
            right_count * 2.0 * math.pi / self.right_cpr]
        rpm_to_radian = 2.0 * math.pi / 60.0
        message.velocity = [left_rpm * rpm_to_radian,
                            right_rpm * rpm_to_radian]
        self.joint_pub.publish(message)

    def publish_odometry(self, stamp, mcu_ms, left_count, right_count):
        if self.previous_left is None:
            self.previous_left = left_count
            self.previous_right = right_count
            self.previous_mcu_ms = mcu_ms
            return
        delta_ms = (mcu_ms - self.previous_mcu_ms) & 0xFFFFFFFF
        left_delta = wrapped_int32_delta(left_count, self.previous_left)
        right_delta = wrapped_int32_delta(right_count, self.previous_right)
        self.previous_left = left_count
        self.previous_right = right_count
        self.previous_mcu_ms = mcu_ms
        if delta_ms == 0 or delta_ms > 1000:
            return
        dt = delta_ms / 1000.0
        left_distance = (left_delta / self.left_cpr) * 2.0 * math.pi * self.radius
        right_distance = (right_delta / self.right_cpr) * 2.0 * math.pi * self.radius
        distance = 0.5 * (left_distance + right_distance)
        delta_yaw = (right_distance - left_distance) / self.separation
        self.x += distance * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += distance * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + delta_yaw),
                              math.cos(self.yaw + delta_yaw))

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation = yaw_quaternion(self.yaw)
        message.twist.twist.linear.x = distance / dt
        message.twist.twist.angular.z = delta_yaw / dt
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.01
        message.twist.covariance[0] = 0.01
        message.twist.covariance[7] = 0.001
        message.twist.covariance[35] = 0.02
        self.odom_pub.publish(message)

    def diagnostic_tick(self):
        now = time.monotonic()
        age = (now - self.last_telemetry_monotonic
               if self.last_telemetry_monotonic else float('inf'))
        status = DiagnosticStatus()
        status.name = 'ESP32 base controller'
        status.hardware_id = 'esp32-uart-c'
        if self.serial_port is None or age > 0.5:
            status.level = DiagnosticStatus.ERROR
            status.message = 'No fresh ESP32 telemetry'
        elif self.mcu_fault:
            status.level = DiagnosticStatus.ERROR
            names = {1: 'IMU', 2: 'LEFT_ENCODER_STALL',
                     3: 'RIGHT_ENCODER_STALL', 4: 'ENCODER_DIRECTION'}
            fault_name = names.get(self.mcu_fault, 'UNKNOWN')
            status.message = f'ESP32 fault {self.mcu_fault}: {fault_name}'
        elif self.motion_guard.fault:
            status.level = DiagnosticStatus.ERROR
            status.message = self.motion_guard.fault
        elif self.motion_guard.pre_stall:
            status.level = DiagnosticStatus.WARN
            status.message = f'PRE_STALL: {self.motion_guard.pre_stall}'
        elif self.motion_guard.recovery_in_progress:
            status.level = DiagnosticStatus.WARN
            status.message = 'Wheel feedback recovery in progress'
        elif not self.motors_enabled:
            status.level = DiagnosticStatus.WARN
            status.message = 'Healthy; motors locked for bench validation'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Healthy'
        status.values = [
            KeyValue(key='port', value=self.port),
            KeyValue(key='telemetry_age_s', value=f'{age:.3f}'),
            KeyValue(key='telemetry_count', value=str(self.telemetry_count)),
            KeyValue(key='parse_errors', value=str(self.parse_errors)),
            KeyValue(key='mcu_flags', value=str(self.mcu_flags)),
            KeyValue(key='mcu_fault', value=str(self.mcu_fault)),
            KeyValue(key='host_motion_fault', value=self.motion_guard.fault),
            KeyValue(key='host_motion_state', value=self.motion_guard.state),
            KeyValue(
                key='host_motion_reason',
                value=(self.motion_guard.pre_stall
                       or self.motion_guard.recovery_reason)),
            KeyValue(key='imu_temperature_c',
                     value=f'{self.temperature:.2f}'),
            KeyValue(key='motors_enabled', value=str(self.motors_enabled)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)

    def destroy_node(self):
        if self.serial_port is not None:
            self.write_line('STOP')
            if self.serial_port is not None:
                os.close(self.serial_port)
                self.serial_port = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
