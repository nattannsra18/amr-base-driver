"""Short LaserScan-guarded reverse used only to escape a blocked start pose."""

from __future__ import annotations

import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


def robust_rear_clearance(
    ranges,
    *,
    angle_min,
    angle_increment,
    range_min,
    range_max,
):
    """Return a conservative tenth-percentile clearance behind the robot."""
    values = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = math.atan2(
            math.sin(angle_min + index * angle_increment),
            math.cos(angle_min + index * angle_increment),
        )
        if abs(math.degrees(angle)) >= 150.0:
            values.append(distance)
    if len(values) < 3:
        return 0.0
    values.sort()
    return values[min(len(values) - 1, int(len(values) * 0.10))]


class GuardedReverse(Node):
    """Expose one fixed, slow reverse with independent live scan protection."""

    def __init__(self):
        super().__init__('guarded_reverse')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/wheel/odometry')
        self.declare_parameter('speed', 0.05)
        self.declare_parameter('distance', 0.10)
        self.declare_parameter('start_clearance', 0.45)
        self.declare_parameter('stop_clearance', 0.35)
        self.declare_parameter('sensor_timeout', 0.50)
        self.declare_parameter('time_limit', 3.0)

        self.speed = min(0.07, max(0.02, float(
            self.get_parameter('speed').value)))
        self.distance = min(0.12, max(0.04, float(
            self.get_parameter('distance').value)))
        self.start_clearance = max(0.40, float(
            self.get_parameter('start_clearance').value))
        self.stop_clearance = max(0.25, min(
            self.start_clearance,
            float(self.get_parameter('stop_clearance').value),
        ))
        self.sensor_timeout = min(1.0, max(0.20, float(
            self.get_parameter('sensor_timeout').value)))
        self.time_limit = min(4.0, max(1.0, float(
            self.get_parameter('time_limit').value)))

        self.data_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.latest_scan = None
        self.scan_monotonic = 0.0
        self.latest_odom = None
        self.odom_monotonic = 0.0
        callbacks = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(Twist, '/cmd_vel_escape', 10)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.scan_callback,
            qos_profile_sensor_data,
            callback_group=callbacks,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            qos_profile_sensor_data,
            callback_group=callbacks,
        )
        self.create_service(
            Trigger,
            '/guarded_reverse_escape',
            self.reverse_callback,
            callback_group=callbacks,
        )

    def scan_callback(self, message):
        clearance = robust_rear_clearance(
            message.ranges,
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
        )
        with self.data_lock:
            self.latest_scan = clearance
            self.scan_monotonic = time.monotonic()

    def odom_callback(self, message):
        position = message.pose.pose.position
        with self.data_lock:
            self.latest_odom = (float(position.x), float(position.y))
            self.odom_monotonic = time.monotonic()

    def snapshot(self):
        with self.data_lock:
            return (
                self.latest_scan,
                self.scan_monotonic,
                self.latest_odom,
                self.odom_monotonic,
            )

    def publish_stop(self):
        self.publisher.publish(Twist())

    def reverse_callback(self, _request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A guarded reverse is already active'
            return response
        try:
            return self.run_reverse(response)
        finally:
            for _ in range(5):
                self.publish_stop()
                time.sleep(0.05)
            self.operation_lock.release()

    def run_reverse(self, response):
        now = time.monotonic()
        clearance, scan_time, start, odom_time = self.snapshot()
        if clearance is None or now - scan_time > self.sensor_timeout:
            response.success = False
            response.message = 'Guarded reverse refused: rear LaserScan is stale'
            return response
        if start is None or now - odom_time > self.sensor_timeout:
            response.success = False
            response.message = 'Guarded reverse refused: wheel odometry is stale'
            return response
        if clearance < self.start_clearance:
            response.success = False
            response.message = (
                'Guarded reverse refused: rear clearance '
                f'{clearance:.2f} m is below {self.start_clearance:.2f} m'
            )
            return response

        command = Twist()
        command.linear.x = -self.speed
        deadline = now + self.time_limit
        while time.monotonic() < deadline:
            now = time.monotonic()
            clearance, scan_time, position, odom_time = self.snapshot()
            if (
                clearance is None
                or now - scan_time > self.sensor_timeout
                or clearance < self.stop_clearance
            ):
                response.success = False
                response.message = 'Guarded reverse stopped: rear path is no longer safe'
                return response
            if position is None or now - odom_time > self.sensor_timeout:
                response.success = False
                response.message = 'Guarded reverse stopped: wheel odometry became stale'
                return response
            travelled = math.hypot(position[0] - start[0], position[1] - start[1])
            if travelled >= self.distance:
                response.success = True
                response.message = (
                    f'Guarded reverse completed {travelled:.2f} m; '
                    f'rear clearance {clearance:.2f} m'
                )
                return response
            self.publisher.publish(command)
            time.sleep(0.05)

        response.success = False
        response.message = 'Guarded reverse timed out before measured motion completed'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GuardedReverse()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.publish_stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
