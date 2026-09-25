"""Lightweight rolling telemetry recorder for post-incident diagnosis."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import shutil
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import TransitionEvent
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def sector_clearances(message: LaserScan) -> dict[str, float | None]:
    """Return conservative nearest ranges around the chassis."""
    sectors: dict[str, list[float]] = {
        'front': [], 'left': [], 'right': [], 'rear': [],
    }
    for index, raw_distance in enumerate(message.ranges):
        distance = float(raw_distance)
        if (
            not math.isfinite(distance)
            or distance < message.range_min
            or distance > message.range_max
        ):
            continue
        angle = math.atan2(
            math.sin(message.angle_min + index * message.angle_increment),
            math.cos(message.angle_min + index * message.angle_increment),
        )
        degrees = math.degrees(angle)
        if abs(degrees) <= 30.0:
            sectors['front'].append(distance)
        elif 60.0 <= degrees <= 120.0:
            sectors['left'].append(distance)
        elif -120.0 <= degrees <= -60.0:
            sectors['right'].append(distance)
        elif abs(degrees) >= 150.0:
            sectors['rear'].append(distance)
    return {
        name: round(min(values), 4) if values else None
        for name, values in sectors.items()
    }


class RollingJsonlWriter:
    """Keep a bounded number of short JSONL segments on disk."""

    def __init__(self, directory, *, segment_seconds=10.0, max_segments=6):
        self.directory = Path(directory).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.segment_seconds = max(1.0, float(segment_seconds))
        self.max_segments = max(2, int(max_segments))
        self.segment_started = None
        self.handle = None
        self.event_count = 0
        self.last_flush = 0.0

    def write(self, event, *, monotonic=None):
        now = time.monotonic() if monotonic is None else float(monotonic)
        if (
            self.handle is None
            or self.segment_started is None
            or now - self.segment_started >= self.segment_seconds
        ):
            self._rotate(now)
        self.handle.write(json.dumps(event, separators=(',', ':')) + '\n')
        if now - self.last_flush >= 1.0:
            self.handle.flush()
            self.last_flush = now
        self.event_count += 1

    def paths(self):
        return sorted(self.directory.glob('black-box-*.jsonl'))

    def close(self):
        if self.handle is not None:
            self.handle.flush()
            self.handle.close()
            self.handle = None

    def archive(self, directory, *, label='operator-mark'):
        """Copy the current rolling window so rotation cannot erase evidence."""
        if self.handle is not None:
            self.handle.flush()
        destination = Path(directory).expanduser() / (
            f'{label}-{time.time_ns()}'
        )
        destination.mkdir(parents=True, exist_ok=False)
        for path in self.paths():
            shutil.copy2(path, destination / path.name)
        return destination

    def _rotate(self, now):
        self.close()
        stamp = time.time_ns()
        path = self.directory / f'black-box-{stamp}.jsonl'
        self.handle = path.open('a', encoding='utf-8', buffering=64 * 1024)
        self.segment_started = now
        self.last_flush = now
        paths = self.paths()
        for expired in paths[:-self.max_segments]:
            expired.unlink(missing_ok=True)


def header_age_ms(node: Node, header) -> float | None:
    stamp_ns = int(header.stamp.sec) * 1_000_000_000 + int(
        header.stamp.nanosec)
    if stamp_ns <= 0:
        return None
    age = (node.get_clock().now().nanoseconds - stamp_ns) / 1_000_000.0
    return round(max(0.0, age), 3)


def diagnostic_level(value) -> int:
    """Normalize ROS diagnostic levels across Python message runtimes."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        return int(value[0]) if value else 3
    return int(value)


class BlackBoxRecorder(Node):
    """Record the last minute of compact robot state without rosbag overhead."""

    def __init__(self):
        super().__init__('black_box_recorder')
        default_directory = os.environ.get(
            'AMR_BLACK_BOX_DIR',
            '~/.local/state/indoor-delivery-robot/black-box',
        )
        self.declare_parameter('directory', default_directory)
        self.declare_parameter('retention_seconds', 60.0)
        self.declare_parameter('segment_seconds', 10.0)
        self.declare_parameter('max_topic_hz', 5.0)
        self.declare_parameter('scan_topic', '/scan_recovery')
        self.declare_parameter(
            'incident_directory',
            str(Path(default_directory).expanduser().parent / 'incidents'),
        )
        segment_seconds = float(self.get_parameter('segment_seconds').value)
        retention_seconds = float(
            self.get_parameter('retention_seconds').value)
        max_segments = max(2, math.ceil(retention_seconds / segment_seconds))
        self.writer = RollingJsonlWriter(
            str(self.get_parameter('directory').value),
            segment_seconds=segment_seconds,
            max_segments=max_segments,
        )
        self.incident_directory = str(
            self.get_parameter('incident_directory').value)
        self.minimum_interval = 1.0 / max(
            1.0, float(self.get_parameter('max_topic_hz').value))
        self.last_recorded: dict[str, float] = {}
        self.previous_cpu_sample = None

        # Record only the command accepted by the owner arbiter. The current
        # owner and rejected/recovery state are already carried by diagnostics.
        self.create_subscription(
            Twist, '/cmd_vel_safe',
            lambda message: self.record_twist('/cmd_vel_safe', message),
            qos_profile_sensor_data,
        )
        # Serial diagnostics contain target/measured RPM, encoder counts and
        # deltas. Avoid duplicate JointState/raw-odometry DDS subscriptions on
        # the resource-constrained C4; retain the fused odometry used by Nav2.
        self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda message: self.record_odometry('/odometry/filtered', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.record_amcl,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self.record_scan, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self.record_diagnostics, 10)
        self.create_subscription(
            TFMessage, '/tf', self.record_tf, qos_profile_sensor_data)
        for node_name in (
            'map_server', 'amcl', 'controller_server', 'planner_server',
            'velocity_smoother', 'collision_monitor',
            'behavior_server', 'bt_navigator',
        ):
            self.create_subscription(
                TransitionEvent,
                f'/{node_name}/transition_event',
                lambda message, name=node_name: self.record_lifecycle(
                    name, message),
                10,
            )
        self.create_service(Trigger, '/black_box/mark', self.mark)
        self.diagnostics = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self.record_host_health)
        # The Robot Agent marks a diagnostic stale after three seconds.  A
        # one-second heartbeat leaves enough scheduling margin on the ODROID
        # under SLAM/Nav2 load and prevents the status from oscillating.
        self.create_timer(1.0, self.publish_health)
        self.record('session', {'event': 'started'}, force=True)

    def record(self, stream, data, *, force=False):
        now = time.monotonic()
        if (
            not force
            and now - self.last_recorded.get(stream, -math.inf)
            < self.minimum_interval
        ):
            return
        self.last_recorded[stream] = now
        self.writer.write({
            'wall_time': time.time(),
            'monotonic': now,
            'stream': stream,
            'data': data,
        }, monotonic=now)

    def should_record(self, stream):
        return (
            time.monotonic() - self.last_recorded.get(stream, -math.inf)
            >= self.minimum_interval
        )

    def record_twist(self, stream, message):
        if not self.should_record(stream):
            return
        self.record(stream, {
            'linear_x': round(float(message.linear.x), 5),
            'angular_z': round(float(message.angular.z), 5),
        })

    def record_odometry(self, stream, message):
        if not self.should_record(stream):
            return
        pose = message.pose.pose
        twist = message.twist.twist
        self.record(stream, {
            'age_ms': header_age_ms(self, message.header),
            'x': round(float(pose.position.x), 5),
            'y': round(float(pose.position.y), 5),
            'qz': round(float(pose.orientation.z), 6),
            'qw': round(float(pose.orientation.w), 6),
            'linear_x': round(float(twist.linear.x), 5),
            'angular_z': round(float(twist.angular.z), 5),
        })

    def record_amcl(self, message):
        if not self.should_record('/amcl_pose'):
            return
        pose = message.pose.pose
        self.record('/amcl_pose', {
            'age_ms': header_age_ms(self, message.header),
            'x': round(float(pose.position.x), 5),
            'y': round(float(pose.position.y), 5),
            'qz': round(float(pose.orientation.z), 6),
            'qw': round(float(pose.orientation.w), 6),
            'covariance_x': round(float(message.pose.covariance[0]), 6),
            'covariance_y': round(float(message.pose.covariance[7]), 6),
            'covariance_yaw': round(float(message.pose.covariance[35]), 6),
        })

    def record_scan(self, message):
        if not self.should_record('/scan'):
            return
        self.record('/scan', {
            'age_ms': header_age_ms(self, message.header),
            'clearance': sector_clearances(message),
        })

    def record_diagnostics(self, message):
        # /diagnostics has several independent publishers. Throttling all of
        # them with one key can retain serial health while silently dropping
        # command ownership (or the reverse).
        names = sorted(status.name for status in message.status)
        stream = '/diagnostics/' + '|'.join(names)
        if not self.should_record(stream):
            return
        statuses = []
        for status in message.status:
            values = {value.key: value.value for value in status.values}
            statuses.append({
                'name': status.name,
                'level': diagnostic_level(status.level),
                'message': status.message,
                'values': values,
            })
        self.record(stream, {
            'age_ms': header_age_ms(self, message.header),
            'statuses': statuses,
        })

    def record_tf(self, message):
        if not self.should_record('/tf'):
            return
        selected = []
        frames = {'map', 'odom', 'base_footprint', 'base_link'}
        ages = []
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            if parent not in frames and child not in frames:
                continue
            age = header_age_ms(self, transform.header)
            if age is not None:
                ages.append(age)
            selected.append({
                'parent': parent,
                'child': child,
                'x': round(float(transform.transform.translation.x), 5),
                'y': round(float(transform.transform.translation.y), 5),
                'qz': round(float(transform.transform.rotation.z), 6),
                'qw': round(float(transform.transform.rotation.w), 6),
            })
        if selected:
            self.record('/tf', {
                'receive_age_ms': round(max(ages), 3) if ages else None,
                'transforms': selected,
            })

    def record_lifecycle(self, node_name, message):
        self.record(f'/lifecycle/{node_name}', {
            'transition_id': int(message.transition.id),
            'transition': message.transition.label,
            'start_state': message.start_state.label,
            'goal_state': message.goal_state.label,
        }, force=True)

    def record_host_health(self):
        load = os.getloadavg()
        cpu_percent = self.read_cpu_percent()
        memory = {}
        try:
            with open('/proc/meminfo', encoding='ascii') as source:
                for line in source:
                    key, value = line.split(':', 1)
                    if key in {'MemTotal', 'MemAvailable'}:
                        memory[key] = int(value.strip().split()[0])
        except (OSError, ValueError):
            pass
        self.record('host', {
            'load_1m': round(load[0], 3),
            'load_5m': round(load[1], 3),
            'load_15m': round(load[2], 3),
            'cpu_percent': cpu_percent,
            'memory_kib': memory,
        }, force=True)

    def read_cpu_percent(self):
        """Return host CPU use since the previous one-second sample."""
        try:
            with open('/proc/stat', encoding='ascii') as source:
                fields = source.readline().split()[1:]
            values = [int(value) for value in fields]
        except (OSError, ValueError):
            return None
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)
        previous = self.previous_cpu_sample
        self.previous_cpu_sample = (idle, total)
        if previous is None or total <= previous[1]:
            return None
        busy_delta = (total - previous[1]) - (idle - previous[0])
        return round(100.0 * busy_delta / (total - previous[1]), 2)

    def mark(self, _request, response):
        self.record('marker', {'event': 'operator_mark'}, force=True)
        archive = self.writer.archive(self.incident_directory)
        response.success = True
        response.message = str(archive)
        return response

    def publish_health(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'Black-box telemetry'
        status.hardware_id = 'odroid-host'
        status.level = DiagnosticStatus.OK
        status.message = 'Rolling telemetry capture active'
        status.values = [
            KeyValue(key='directory', value=str(self.writer.directory)),
            KeyValue(key='segments', value=str(len(self.writer.paths()))),
            KeyValue(key='events', value=str(self.writer.event_count)),
        ]
        message.status = [status]
        self.diagnostics.publish(message)

    def destroy_node(self):
        self.record('session', {'event': 'stopped'}, force=True)
        self.writer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlackBoxRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
