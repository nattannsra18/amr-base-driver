import binascii
import math
import os
import select
import struct
import termios
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Empty, SetBool, Trigger

from .imu_bias import StationaryBias


TELEMETRY_MAGIC = b'\x5a\xa5'
TELEMETRY_FORMAT = '<HBBIIiihhhhhhhhhHBBH'
TELEMETRY_LENGTH = struct.calcsize(TELEMETRY_FORMAT)


def crc16_ccitt(data):
    return binascii.crc_hqx(data, 0xFFFF)


def wrapped_int32_delta(current, previous):
    delta = (current - previous) & 0xFFFFFFFF
    return delta - 0x100000000 if delta & 0x80000000 else delta


def trim_straight_wheel_rpm(left_rpm, right_rpm, measured_left_rpm,
                            measured_right_rpm, gain, maximum_trim_rpm):
    """Apply a small encoder trim while preserving average forward speed."""
    if left_rpm * right_rpm <= 0.0:
        return left_rpm, right_rpm
    measured_peak = max(abs(measured_left_rpm), abs(measured_right_rpm))
    if measured_peak <= 3.0:
        return left_rpm, right_rpm
    correction = gain * (abs(measured_right_rpm) - abs(measured_left_rpm))
    limit = min(maximum_trim_rpm, 0.9 * min(abs(left_rpm), abs(right_rpm)))
    correction = max(-limit, min(limit, correction))
    direction = math.copysign(1.0, left_rpm)
    return (
        left_rpm + direction * correction,
        right_rpm - direction * correction,
    )


def wheel_is_stalled(target_rpm, measured_rpm, command_age, *,
                     minimum_target_rpm, maximum_measured_rpm,
                     timeout_seconds):
    """Detect a sustained, trackable command with no wheel motion."""
    return (
        abs(float(target_rpm)) >= float(minimum_target_rpm)
        and abs(float(measured_rpm)) <= float(maximum_measured_rpm)
        and float(command_age) >= float(timeout_seconds)
    )


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
        self.declare_parameter('straight_trim_gain', 0.15)
        self.declare_parameter('straight_trim_max_rpm', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.25)
        self.declare_parameter('stall_guard_enabled', True)
        self.declare_parameter('stall_guard_min_target_rpm', 20.0)
        self.declare_parameter('stall_guard_max_measured_rpm', 3.0)
        self.declare_parameter('stall_guard_timeout_seconds', 1.2)
        self.declare_parameter('stall_guard_recent_seconds', 30.0)
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
        self.straight_trim_gain = max(
            0.0, float(self.get_parameter('straight_trim_gain').value))
        self.straight_trim_max_rpm = max(
            0.0, float(self.get_parameter('straight_trim_max_rpm').value))
        self.cmd_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.stall_guard_enabled = bool(
            self.get_parameter('stall_guard_enabled').value)
        self.stall_guard_min_target_rpm = max(
            0.0, float(self.get_parameter('stall_guard_min_target_rpm').value))
        self.stall_guard_max_measured_rpm = max(
            0.0,
            float(self.get_parameter('stall_guard_max_measured_rpm').value),
        )
        self.stall_guard_timeout_seconds = max(
            0.5,
            float(self.get_parameter('stall_guard_timeout_seconds').value),
        )
        self.stall_guard_recent_seconds = max(
            self.stall_guard_timeout_seconds,
            float(self.get_parameter('stall_guard_recent_seconds').value),
        )
        self.telemetry_publish_divisor = max(
            1,
            int(self.get_parameter('telemetry_publish_divisor').value),
        )
        self.motors_enabled = bool(self.get_parameter('enable_motors').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.serial_port = None
        self.serial_lock = threading.RLock()
        self.telemetry_lock = threading.Lock()
        self.reader_stop = threading.Event()
        self.rx_buffer = bytearray()
        self.last_open_attempt = 0.0
        self.last_telemetry_monotonic = 0.0
        self.last_cmd_monotonic = 0.0
        self.last_diag_monotonic = 0.0
        self.command_sequence = 0
        self.telemetry_count = 0
        self.telemetry_revision = 0
        self.published_telemetry_revision = 0
        self.latest_telemetry = None
        self.maximum_telemetry_gap = 0.0
        self.telemetry_gap_overrun_count = 0
        self.previous_telemetry_sequence = None
        self.previous_telemetry_mcu_ms = None
        self.maximum_mcu_telemetry_gap_ms = 0
        self.missed_telemetry_frames = 0
        self.mcu_restart_count = 0
        self.diagnostic_requested = False
        self.parse_errors = 0
        self.mcu_flags = 0
        self.mcu_fault = 0
        self.temperature = float('nan')
        self.requested_linear = 0.0
        self.requested_angular = 0.0
        self.target_left_rpm = 0.0
        self.target_right_rpm = 0.0
        self.measured_left_rpm = 0.0
        self.measured_right_rpm = 0.0
        self.encoder_left_count = 0
        self.encoder_right_count = 0
        self.encoder_left_delta = 0
        self.encoder_right_delta = 0
        self.wheel_linear_mps = 0.0
        self.wheel_angular_rps = 0.0
        self.imu_angular_z_rps = 0.0
        self.imu_bias_estimator = StationaryBias()
        self.imu_samples = 0
        self.last_imu_motion_monotonic = -100.0
        self.drive_blocked = False
        self.blocked_left = False
        self.blocked_right = False
        self.stall_candidate_started = [None, None]
        self.last_drive_stall_monotonic = 0.0
        self.blocked_target_rpm = (0.0, 0.0)
        self.previous_left = None
        self.previous_right = None
        self.previous_mcu_ms = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # These are live sensor streams, not durable state. Best-effort keeps
        # a slow DDS reader from blocking the callback that drains the UART.
        self.imu_pub = self.create_publisher(
            Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.corrected_imu_pub = self.create_publisher(
            Imu, 'imu/data', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(
            Odometry, 'wheel/odometry', qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 20)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_service(Empty, 'reset_wheel_odometry', self.reset_odom)
        self.create_service(
            SetBool, 'set_motors_enabled', self.set_motors_enabled)
        self.create_service(
            Trigger, 'clear_motor_fault', self.clear_motor_fault)
        # UART reading is isolated from ROS/DDS publishing so executor load
        # cannot make healthy ESP32 telemetry appear stale.
        self.reader_thread = threading.Thread(
            target=self.serial_reader_loop,
            name='esp32-telemetry-reader',
            daemon=True,
        )
        self.reader_thread.start()
        self.create_timer(0.02, self.publish_telemetry_tick)
        self.create_timer(0.10, self.command_tick)
        # Normal diagnostics are low-rate to keep DDS/JSON merging off the C4
        # hot path. Fault and pre-stall transitions publish immediately below.
        self.create_timer(1.0, self.diagnostic_tick)

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
        self.clear_drive_blocked()
        self.write_line('STOP')
        self.write_line('CLEAR')
        response.success = True
        response.message = (
            'STOP and CLEAR sent; verify diagnostics mcu_fault=0 before motion')
        return response

    def set_motors_enabled(self, request, response):
        """Arm or lock motor output without restarting localization/Nav2."""
        self.requested_linear = 0.0
        self.requested_angular = 0.0
        self.last_cmd_monotonic = 0.0
        self.clear_drive_blocked()
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
            with self.serial_lock:
                self.serial_port = descriptor
                self.rx_buffer.clear()
            self.get_logger().info('ESP32 serial port connected')
        except OSError as error:
            self.serial_port = None
            self.get_logger().warning(f'Waiting for ESP32 serial port: {error}')

    def close_serial(self, error):
        with self.serial_lock:
            if self.serial_port is not None:
                try:
                    os.close(self.serial_port)
                except OSError:
                    pass
            self.serial_port = None
        self.get_logger().error(f'ESP32 serial disconnected: {error}')

    def serial_reader_loop(self):
        """Drain UART independently from the ROS executor."""
        while not self.reader_stop.is_set():
            with self.serial_lock:
                descriptor = self.serial_port
            if descriptor is None:
                self.open_serial()
                self.reader_stop.wait(0.02)
                continue
            try:
                readable, _, _ = select.select([descriptor], [], [], 0.05)
                if not readable:
                    continue
                data = os.read(descriptor, 4096)
                if not data:
                    raise OSError('ESP32 UART returned EOF')
                self.rx_buffer.extend(data)
                self.extract_telemetry_frames()
                if len(self.rx_buffer) > 4096:
                    self.rx_buffer.clear()
                    self.parse_errors += 1
            except (OSError, ValueError) as error:
                if not self.reader_stop.is_set():
                    self.close_serial(error)

    def publish_telemetry_tick(self):
        with self.telemetry_lock:
            if self.telemetry_revision == self.published_telemetry_revision:
                return
            self.published_telemetry_revision = self.telemetry_revision
            telemetry = self.latest_telemetry
            publish_diagnostic = self.diagnostic_requested
            self.diagnostic_requested = False
        if telemetry is None:
            return
        if publish_diagnostic:
            self.diagnostic_tick()
        if telemetry['count'] % self.telemetry_publish_divisor:
            return
        stamp = self.get_clock().now().to_msg()
        self.publish_imu(stamp, telemetry['accel'], telemetry['gyro'])
        self.publish_joint_state(
            stamp,
            telemetry['left_count'],
            telemetry['right_count'],
            telemetry['left_rpm'],
            telemetry['right_rpm'],
        )
        self.publish_odometry(
            stamp,
            telemetry['mcu_ms'],
            telemetry['left_count'],
            telemetry['right_count'],
        )

    def write_line(self, line):
        with self.serial_lock:
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
                   self.mcu_fault == 0)
        if not healthy:
            self.target_left_rpm = 0.0
            self.target_right_rpm = 0.0
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
            # Preserve differential-drive curvature.  Independent wheel floors
            # turned shallow arcs into pivots and made Nav2 correct repeatedly.
            if abs(self.requested_angular) <= 0.03:
                left_rpm, right_rpm = trim_straight_wheel_rpm(
                    left_rpm,
                    right_rpm,
                    self.measured_left_rpm,
                    self.measured_right_rpm,
                    self.straight_trim_gain,
                    self.straight_trim_max_rpm,
                )
                peak = max(abs(left_rpm), abs(right_rpm))
                if peak > self.max_rpm:
                    left_rpm *= self.max_rpm / peak
                    right_rpm *= self.max_rpm / peak
        if self.drive_blocked:
            old_left, old_right = self.blocked_target_rpm
            reversed_direction = any(
                old * new < 0.0
                for old, new in ((old_left, left_rpm), (old_right, right_rpm))
            )
            stopped = max(abs(left_rpm), abs(right_rpm)) < 0.5
            if stopped or reversed_direction:
                self.clear_drive_blocked()
            else:
                self.target_left_rpm = left_rpm
                self.target_right_rpm = right_rpm
                self.write_line('STOP')
                return

        self.target_left_rpm = left_rpm
        self.target_right_rpm = right_rpm
        self.update_stall_guard(now, left_rpm, right_rpm)
        if self.drive_blocked:
            self.write_line('STOP')
            return
        self.write_line(
            f'CMD,{self.command_sequence},{left_rpm:.3f},{right_rpm:.3f}')
        self.command_sequence = (self.command_sequence + 1) & 0xFFFFFFFF

    def clear_drive_blocked(self):
        self.drive_blocked = False
        self.blocked_left = False
        self.blocked_right = False
        self.stall_candidate_started = [None, None]
        self.blocked_target_rpm = (0.0, 0.0)

    def update_stall_guard(self, now, left_rpm, right_rpm):
        if not self.stall_guard_enabled:
            self.stall_candidate_started = [None, None]
            return
        targets = (left_rpm, right_rpm)
        measured = (self.measured_left_rpm, self.measured_right_rpm)
        stalled = [False, False]
        for index, (target, actual) in enumerate(zip(targets, measured)):
            if (
                abs(target) < self.stall_guard_min_target_rpm
                or abs(actual) > self.stall_guard_max_measured_rpm
            ):
                self.stall_candidate_started[index] = None
                continue
            started = self.stall_candidate_started[index]
            if started is None:
                self.stall_candidate_started[index] = now
                continue
            stalled[index] = wheel_is_stalled(
                target,
                actual,
                now - started,
                minimum_target_rpm=self.stall_guard_min_target_rpm,
                maximum_measured_rpm=self.stall_guard_max_measured_rpm,
                timeout_seconds=self.stall_guard_timeout_seconds,
            )
        if not any(stalled):
            return
        self.drive_blocked = True
        self.blocked_left, self.blocked_right = stalled
        self.blocked_target_rpm = targets
        self.last_drive_stall_monotonic = now
        sides = '/'.join(
            side for side, is_stalled in zip(('left', 'right'), stalled)
            if is_stalled
        )
        self.get_logger().error(
            f'Encoder stall guard stopped both motors: {sides} wheel did not move'
        )
        self.diagnostic_tick()

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
        sequence = values[3]
        left_count, right_count = values[5:7]
        left_rpm, right_rpm = (value / 100.0 for value in values[7:9])
        accel = [value / 1000.0 for value in values[9:12]]
        gyro = [value / 100.0 for value in values[12:15]]
        self.imu_angular_z_rps = gyro[2] * math.pi / 180.0
        self.temperature = values[15] / 100.0
        received_at = time.monotonic()
        with self.telemetry_lock:
            previous_mcu_fault = self.mcu_fault
            if self.last_telemetry_monotonic:
                gap = received_at - self.last_telemetry_monotonic
                self.maximum_telemetry_gap = max(
                    self.maximum_telemetry_gap, gap)
                if gap > 0.3:
                    self.telemetry_gap_overrun_count += 1
            telemetry_restarted = (
                self.previous_telemetry_sequence is not None
                and self.previous_telemetry_mcu_ms is not None
                and sequence < self.previous_telemetry_sequence
                and mcu_ms < self.previous_telemetry_mcu_ms
            )
            if telemetry_restarted:
                self.mcu_restart_count += 1
            else:
                if self.previous_telemetry_sequence is not None:
                    sequence_delta = (
                        sequence - self.previous_telemetry_sequence
                    ) & 0xFFFFFFFF
                    self.missed_telemetry_frames += max(
                        0, sequence_delta - 1)
                if self.previous_telemetry_mcu_ms is not None:
                    mcu_gap_ms = (
                        mcu_ms - self.previous_telemetry_mcu_ms
                    ) & 0xFFFFFFFF
                    self.maximum_mcu_telemetry_gap_ms = max(
                        self.maximum_mcu_telemetry_gap_ms,
                        mcu_gap_ms,
                    )
            self.previous_telemetry_sequence = sequence
            self.previous_telemetry_mcu_ms = mcu_ms
            self.mcu_flags = values[16]
            self.mcu_fault = values[17]
            self.encoder_left_count = left_count
            self.encoder_right_count = right_count
            self.measured_left_rpm = left_rpm
            self.measured_right_rpm = right_rpm
            self.last_telemetry_monotonic = received_at
            self.telemetry_count += 1
            self.telemetry_revision += 1
            self.latest_telemetry = {
                'count': self.telemetry_count,
                'mcu_ms': mcu_ms,
                'left_count': left_count,
                'right_count': right_count,
                'left_rpm': left_rpm,
                'right_rpm': right_rpm,
                'accel': accel,
                'gyro': gyro,
            }
            if self.mcu_fault != previous_mcu_fault:
                self.diagnostic_requested = True

    def publish_imu(self, stamp, accel_g, gyro_dps):
        degree_to_radian = math.pi / 180.0
        gyro = [value * degree_to_radian for value in gyro_dps]
        gravity = 9.80665 * self.accel_scale
        acceleration = [value * gravity for value in accel_g]

        raw = self.make_imu_message(stamp, gyro, acceleration, 2.5e-5)
        self.imu_pub.publish(raw)

        now = time.monotonic()
        moving = (
            abs(self.wheel_linear_mps) > 0.001
            or abs(self.wheel_angular_rps) > 0.001
        )
        if moving:
            self.last_imu_motion_monotonic = now
        command_recent = (
            self.last_cmd_monotonic > 0.0
            and now - self.last_cmd_monotonic < 0.5
        )
        commanded = command_recent and (
            abs(self.requested_linear) > 0.001
            or abs(self.requested_angular) > 0.001
        )
        stationary = (
            now - self.last_imu_motion_monotonic > 2.0
            and not commanded
        )
        acceleration_norm = math.sqrt(sum(value * value for value in acceleration))
        bias = self.imu_bias_estimator.update(
            now, gyro, acceleration_norm, stationary)
        corrected_gyro = [value - offset for value, offset in zip(gyro, bias)]
        variance = 2.5e-5 if self.imu_bias_estimator.ready else 4.0e-4
        corrected = self.make_imu_message(
            stamp, corrected_gyro, acceleration, variance)
        self.corrected_imu_pub.publish(corrected)
        self.imu_samples += 1

    def make_imu_message(self, stamp, gyro, acceleration, gyro_variance):
        """Build one IMU sample without a DDS subscribe/republish process."""
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self.imu_frame
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = gyro[0]
        message.angular_velocity.y = gyro[1]
        message.angular_velocity.z = gyro[2]
        message.angular_velocity_covariance = [
            gyro_variance, 0.0, 0.0,
            0.0, gyro_variance, 0.0,
            0.0, 0.0, gyro_variance]
        message.linear_acceleration.x = acceleration[0]
        message.linear_acceleration.y = acceleration[1]
        message.linear_acceleration.z = acceleration[2]
        message.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04]
        return message

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
        self.encoder_left_delta = left_delta
        self.encoder_right_delta = right_delta
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
        self.wheel_linear_mps = distance / dt
        self.wheel_angular_rps = delta_yaw / dt
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

        serial = DiagnosticStatus()
        serial.name = 'Base serial telemetry'
        serial.hardware_id = 'odroid-uart-c'
        if self.serial_port is None or age > 0.5:
            serial.level = DiagnosticStatus.ERROR
            serial.message = 'No fresh ESP32 telemetry'
        else:
            serial.level = DiagnosticStatus.OK
            serial.message = 'Fresh telemetry'
        serial.values = [
            KeyValue(key='port', value=self.port),
            KeyValue(key='telemetry_age_s', value=f'{age:.3f}'),
            KeyValue(key='telemetry_count', value=str(self.telemetry_count)),
            KeyValue(key='parse_errors', value=str(self.parse_errors)),
            KeyValue(
                key='maximum_telemetry_gap_s',
                value=f'{self.maximum_telemetry_gap:.3f}'),
            KeyValue(
                key='telemetry_gap_overrun_count',
                value=str(self.telemetry_gap_overrun_count)),
            KeyValue(
                key='maximum_mcu_telemetry_gap_ms',
                value=str(self.maximum_mcu_telemetry_gap_ms)),
            KeyValue(
                key='missed_telemetry_frames',
                value=str(self.missed_telemetry_frames)),
            KeyValue(
                key='mcu_restart_count',
                value=str(self.mcu_restart_count)),
        ]

        mcu = DiagnosticStatus()
        mcu.name = 'ESP32 MCU fault'
        mcu.hardware_id = 'esp32-uart-c'
        if self.mcu_fault:
            mcu.level = DiagnosticStatus.ERROR
            names = {1: 'IMU', 2: 'LEFT_ENCODER_STALL',
                     3: 'RIGHT_ENCODER_STALL', 4: 'ENCODER_DIRECTION'}
            fault_name = names.get(self.mcu_fault, 'UNKNOWN')
            mcu.message = f'ESP32 fault {self.mcu_fault}: {fault_name}'
        else:
            mcu.level = DiagnosticStatus.OK
            mcu.message = 'No MCU fault'
        mcu.values = [
            KeyValue(key='mcu_flags', value=str(self.mcu_flags)),
            KeyValue(key='mcu_fault', value=str(self.mcu_fault)),
            KeyValue(key='imu_temperature_c', value=f'{self.temperature:.2f}'),
        ]

        drive = DiagnosticStatus()
        drive.name = 'Base drive telemetry'
        drive.hardware_id = 'odroid-base-driver'
        stall_age = (
            now - self.last_drive_stall_monotonic
            if self.last_drive_stall_monotonic else float('inf')
        )
        stall_recent = stall_age <= self.stall_guard_recent_seconds
        drive.level = (
            DiagnosticStatus.ERROR if self.drive_blocked else DiagnosticStatus.OK
        )
        drive.message = (
            'Encoder stall guard is holding motor output at zero'
            if self.drive_blocked else 'Wheel telemetry available'
        )
        drive.values = [
            KeyValue(key='motors_enabled', value=str(self.motors_enabled)),
            KeyValue(
                key='target_left_rpm', value=f'{self.target_left_rpm:.3f}'),
            KeyValue(
                key='target_right_rpm', value=f'{self.target_right_rpm:.3f}'),
            KeyValue(
                key='measured_left_rpm', value=f'{self.measured_left_rpm:.3f}'),
            KeyValue(
                key='measured_right_rpm', value=f'{self.measured_right_rpm:.3f}'),
            KeyValue(
                key='encoder_left_count', value=str(self.encoder_left_count)),
            KeyValue(
                key='encoder_right_count', value=str(self.encoder_right_count)),
            KeyValue(
                key='encoder_left_delta', value=str(self.encoder_left_delta)),
            KeyValue(
                key='encoder_right_delta', value=str(self.encoder_right_delta)),
            KeyValue(
                key='requested_linear_mps',
                value=f'{self.requested_linear:.4f}'),
            KeyValue(
                key='requested_angular_rps',
                value=f'{self.requested_angular:.4f}'),
            KeyValue(
                key='wheel_linear_mps', value=f'{self.wheel_linear_mps:.4f}'),
            KeyValue(
                key='wheel_angular_rps', value=f'{self.wheel_angular_rps:.4f}'),
            KeyValue(
                key='imu_angular_z_rps',
                value=f'{self.imu_angular_z_rps:.4f}'),
            KeyValue(key='drive_blocked', value=str(self.drive_blocked)),
            KeyValue(key='blocked_left', value=str(self.blocked_left)),
            KeyValue(key='blocked_right', value=str(self.blocked_right)),
            KeyValue(key='drive_stall_recent', value=str(stall_recent)),
            KeyValue(
                key='drive_stall_age_s',
                value=f'{stall_age:.3f}' if math.isfinite(stall_age) else 'inf'),
        ]

        imu = DiagnosticStatus()
        imu.name = 'MPU6050 residual bias'
        imu.hardware_id = 'mpu6050'
        imu.level = (
            DiagnosticStatus.OK
            if self.imu_bias_estimator.ready else DiagnosticStatus.WARN
        )
        imu.message = (
            'Stationary bias calibrated'
            if self.imu_bias_estimator.ready
            else 'Waiting for five seconds of stable stationary data'
        )
        imu.values = [KeyValue(key='samples', value=str(self.imu_samples))]
        for axis, bias in zip('xyz', self.imu_bias_estimator.bias):
            imu.values.append(KeyValue(
                key=f'gyro_bias_{axis}_rad_s', value=f'{bias:.6f}'))

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [serial, mcu, drive, imu]
        self.diag_pub.publish(array)

    def destroy_node(self):
        self.reader_stop.set()
        self.write_line('STOP')
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        with self.serial_lock:
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
