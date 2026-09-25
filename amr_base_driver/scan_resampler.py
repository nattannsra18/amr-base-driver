"""Angle-aware fixed-bin LaserScan resampling for the YDLidar X3."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def resample_ranges(angle_min, angle_increment, ranges, output_beams,
                    range_min, range_max):
    """Place raw samples into fixed angular bins, retaining nearest obstacle."""
    output = [math.inf] * output_beams
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
        return output
    output_increment = 2.0 * math.pi / output_beams
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        angle = angle_min + index * angle_increment
        normalized = (angle + math.pi) % (2.0 * math.pi)
        output_index = int(round(normalized / output_increment)) % output_beams
        if value < output[output_index]:
            output[output_index] = value
    return output


def has_valid_scan_metadata(scan):
    """Return whether scan metadata is safe to forward to navigation."""
    return (
        math.isfinite(scan.angle_min) and
        math.isfinite(scan.angle_increment) and
        scan.angle_increment != 0.0 and
        math.isfinite(scan.range_min) and
        math.isfinite(scan.range_max) and
        0.0 <= scan.range_min < scan.range_max and
        math.isfinite(scan.scan_time) and
        scan.scan_time >= 0.0
    )


class ScanResampler(Node):
    def __init__(self):
        super().__init__('scan_resampler')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('telemetry_topic', '/scan_recovery')
        self.declare_parameter('telemetry_hz', 4.0)
        self.declare_parameter('output_beams', 360)
        self.output_beams = int(self.get_parameter('output_beams').value)
        if self.output_beams < 90:
            raise ValueError('output_beams must be at least 90')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        telemetry_topic = self.get_parameter('telemetry_topic').value
        telemetry_hz = max(
            1.0, float(self.get_parameter('telemetry_hz').value))
        self.telemetry_interval = 1.0 / telemetry_hz
        self.last_telemetry_publish = -math.inf
        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        self.telemetry_publisher = self.create_publisher(
            LaserScan, telemetry_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, input_topic, self.callback, qos_profile_sensor_data)
        self.input_count = 0
        self.get_logger().info(
            f'Angle-aware scan resampling: {input_topic} -> {output_topic}; '
            f'{self.output_beams} beams; {telemetry_topic} at '
            f'{telemetry_hz:.1f} Hz')

    def callback(self, raw):
        if not has_valid_scan_metadata(raw):
            self.get_logger().warning(
                'Dropping LaserScan with invalid metadata',
                throttle_duration_sec=5.0)
            return
        output = LaserScan()
        output.header = raw.header
        output.angle_min = -math.pi
        output.angle_increment = 2.0 * math.pi / self.output_beams
        output.angle_max = (
            output.angle_min + (self.output_beams - 1) *
            output.angle_increment)
        output.scan_time = raw.scan_time
        output.time_increment = (
            raw.scan_time / self.output_beams if raw.scan_time > 0.0 else 0.0)
        output.range_min = raw.range_min
        output.range_max = raw.range_max
        output.ranges = resample_ranges(
            raw.angle_min, raw.angle_increment, raw.ranges,
            self.output_beams, raw.range_min, raw.range_max)
        # X3 is configured without intensity, so do not fabricate values.
        output.intensities = []
        self.publisher.publish(output)
        now = time.monotonic()
        if now - self.last_telemetry_publish >= self.telemetry_interval:
            self.telemetry_publisher.publish(output)
            self.last_telemetry_publish = now
        self.input_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ScanResampler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
