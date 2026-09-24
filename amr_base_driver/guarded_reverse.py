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


FOOTPRINT_MIN_X = -0.087
FOOTPRINT_MAX_X = 0.303
FOOTPRINT_MIN_Y = -0.190
FOOTPRINT_MAX_Y = 0.190
FOOTPRINT_PADDING = 0.020
LIDAR_X = -0.042
LIDAR_Y = -0.005


def adjacent_cluster_clearance(samples, *, beam_count=None):
    """Return the nearest two-beam obstacle cluster, ignoring one speckle."""
    ordered = sorted(samples, key=lambda item: item[0])
    candidates = [
        max(previous[1], current[1])
        for previous, current in zip(ordered, ordered[1:])
        if current[0] == previous[0] + 1
    ]
    if (
        beam_count
        and len(ordered) >= 2
        and ordered[0][0] == 0
        and ordered[-1][0] == beam_count - 1
    ):
        candidates.append(max(ordered[0][1], ordered[-1][1]))
    return min(candidates) if candidates else None


def robust_rear_clearance(
    ranges,
    *,
    angle_min,
    angle_increment,
    range_min,
    range_max,
):
    """Return the nearest adjacent-beam obstacle cluster behind the robot."""
    values = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if (
            not math.isfinite(distance)
            or not range_min <= distance <= range_max
        ):
            continue
        angle = math.atan2(
            math.sin(angle_min + index * angle_increment),
            math.cos(angle_min + index * angle_increment),
        )
        if abs(math.degrees(angle)) >= 150.0:
            values.append((index, distance))
    clearance = adjacent_cluster_clearance(
        values,
        beam_count=len(ranges),
    )
    return 0.0 if clearance is None else clearance


def robust_turn_side_clearance(
    ranges,
    *,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    turn_direction,
):
    """
    Return conservative clearance on the side toward an arc's turn.

    ``turn_direction`` follows ROS angular velocity convention: positive is a
    left (counter-clockwise) turn and negative is a right turn.  The sector is
    deliberately broad so a reverse arc protects both the side of the chassis
    and its rear corner.
    """
    if turn_direction not in (-1, 1):
        raise ValueError('turn_direction must be -1 or 1')

    values = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if (
            not math.isfinite(distance)
            or not range_min <= distance <= range_max
        ):
            continue
        angle = math.atan2(
            math.sin(angle_min + index * angle_increment),
            math.cos(angle_min + index * angle_increment),
        )
        signed_degrees = math.degrees(angle) * turn_direction
        if 70.0 <= signed_degrees <= 155.0:
            values.append((index, distance))
    clearance = adjacent_cluster_clearance(
        values,
        beam_count=len(ranges),
    )
    return 0.0 if clearance is None else clearance


def recovery_sector_clearances(
    ranges,
    *,
    angle_min,
    angle_increment,
    range_min,
    range_max,
):
    """Compute rear, left, and right clustered clearances in one scan pass."""
    rear = []
    left = []
    right = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if (
            not math.isfinite(distance)
            or not range_min <= distance <= range_max
        ):
            continue
        angle = math.atan2(
            math.sin(angle_min + index * angle_increment),
            math.cos(angle_min + index * angle_increment),
        )
        degrees = math.degrees(angle)
        sample = (index, distance)
        if abs(degrees) >= 150.0:
            rear.append(sample)
        if 70.0 <= degrees <= 155.0:
            left.append(sample)
        elif -155.0 <= degrees <= -70.0:
            right.append(sample)

    beam_count = len(ranges)
    rear_clearance = adjacent_cluster_clearance(rear, beam_count=beam_count)
    left_clearance = adjacent_cluster_clearance(left, beam_count=beam_count)
    right_clearance = adjacent_cluster_clearance(right, beam_count=beam_count)
    return (
        0.0 if rear_clearance is None else rear_clearance,
        0.0 if left_clearance is None else left_clearance,
        0.0 if right_clearance is None else right_clearance,
    )


def swept_arc_obstacle_distance(
    ranges,
    *,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    turn_direction,
    arc_speed=0.05,
    arc_turn_rate=0.18,
    arc_yaw=0.22,
):
    """
    Return a clustered obstacle intersecting the short arc footprint.

    Laser points are converted into ``base_footprint`` coordinates using the
    measured LiDAR offset. The physical footprint is sampled along the entire
    commanded reverse arc. A single isolated beam is ignored, while any two
    adjacent beams intersecting the sweep stop the maneuver.
    """
    if turn_direction not in (-1, 1):
        raise ValueError('turn_direction must be -1 or 1')
    speed = max(0.001, abs(float(arc_speed)))
    turn_rate = max(0.001, abs(float(arc_turn_rate)))
    target_yaw = max(0.01, abs(float(arc_yaw)))
    angular_velocity = turn_direction * turn_rate
    linear_velocity = -speed
    radius = linear_velocity / angular_velocity
    steps = max(6, int(math.ceil(target_yaw / 0.02)))
    poses = []
    for step in range(1, steps + 1):
        yaw = turn_direction * target_yaw * step / steps
        poses.append((
            radius * math.sin(yaw),
            radius * (1.0 - math.cos(yaw)),
            yaw,
        ))

    collisions = []
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if (
            not math.isfinite(distance)
            or not range_min <= distance <= range_max
        ):
            continue
        angle = angle_min + index * angle_increment
        point_x = LIDAR_X + distance * math.cos(angle)
        point_y = LIDAR_Y + distance * math.sin(angle)
        for pose_x, pose_y, pose_yaw in poses:
            cosine = math.cos(pose_yaw)
            sine = math.sin(pose_yaw)
            relative_x = (
                cosine * (point_x - pose_x)
                + sine * (point_y - pose_y)
            )
            relative_y = (
                -sine * (point_x - pose_x)
                + cosine * (point_y - pose_y)
            )
            if (
                FOOTPRINT_MIN_X - FOOTPRINT_PADDING
                <= relative_x
                <= FOOTPRINT_MAX_X + FOOTPRINT_PADDING
                and FOOTPRINT_MIN_Y - FOOTPRINT_PADDING
                <= relative_y
                <= FOOTPRINT_MAX_Y + FOOTPRINT_PADDING
            ):
                collisions.append((index, distance))
                break
    return adjacent_cluster_clearance(
        collisions,
        beam_count=len(ranges),
    )


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def measured_arc_progress(start_pose, current_pose, turn_direction):
    """Return measured translation and yaw in the requested turn direction."""
    if turn_direction not in (-1, 1):
        raise ValueError('turn_direction must be -1 or 1')
    travelled = math.hypot(
        current_pose[0] - start_pose[0],
        current_pose[1] - start_pose[1],
    )
    directed_yaw = (
        normalize_angle(current_pose[2] - start_pose[2]) * turn_direction
    )
    return travelled, directed_yaw


def arc_motion_complete(
    start_pose,
    current_pose,
    *,
    turn_direction,
    target_distance,
    target_yaw,
):
    """Require odometry to confirm both translation and directed rotation."""
    travelled, directed_yaw = measured_arc_progress(
        start_pose,
        current_pose,
        turn_direction,
    )
    return travelled >= target_distance and directed_yaw >= target_yaw


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
        self.declare_parameter('arc_speed', 0.05)
        self.declare_parameter('arc_turn_rate', 0.18)
        self.declare_parameter('arc_distance', 0.07)
        self.declare_parameter('arc_yaw', 0.22)
        self.declare_parameter('arc_start_rear_clearance', 0.32)
        self.declare_parameter('arc_stop_rear_clearance', 0.24)
        self.declare_parameter('arc_start_side_clearance', 0.40)
        self.declare_parameter('arc_stop_side_clearance', 0.28)
        self.declare_parameter('arc_time_limit', 3.0)

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
        self.arc_speed = min(0.07, max(0.03, float(
            self.get_parameter('arc_speed').value)))
        self.arc_turn_rate = min(0.25, max(0.12, float(
            self.get_parameter('arc_turn_rate').value)))
        self.arc_distance = min(0.14, max(0.06, float(
            self.get_parameter('arc_distance').value)))
        self.arc_yaw = min(0.45, max(0.18, float(
            self.get_parameter('arc_yaw').value)))
        self.arc_start_rear_clearance = max(0.30, float(
            self.get_parameter('arc_start_rear_clearance').value))
        self.arc_stop_rear_clearance = max(0.20, min(
            self.arc_start_rear_clearance,
            float(self.get_parameter('arc_stop_rear_clearance').value),
        ))
        self.arc_start_side_clearance = max(0.35, float(
            self.get_parameter('arc_start_side_clearance').value))
        self.arc_stop_side_clearance = max(0.22, min(
            self.arc_start_side_clearance,
            float(self.get_parameter('arc_stop_side_clearance').value),
        ))
        self.arc_time_limit = min(5.0, max(1.5, float(
            self.get_parameter('arc_time_limit').value)))

        self.data_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.latest_scan = None
        self.latest_left_clearance = None
        self.latest_right_clearance = None
        self.latest_scan_geometry = None
        self.scan_monotonic = 0.0
        self.latest_odom = None
        self.latest_yaw = None
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
        self.create_service(
            Trigger,
            '/guarded_arc_escape_left',
            self.arc_left_callback,
            callback_group=callbacks,
        )
        self.create_service(
            Trigger,
            '/guarded_arc_escape_right',
            self.arc_right_callback,
            callback_group=callbacks,
        )

    def scan_callback(self, message):
        clearance, left_clearance, right_clearance = (
            recovery_sector_clearances(
                message.ranges,
                angle_min=message.angle_min,
                angle_increment=message.angle_increment,
                range_min=message.range_min,
                range_max=message.range_max,
            )
        )
        with self.data_lock:
            self.latest_scan = clearance
            self.latest_left_clearance = left_clearance
            self.latest_right_clearance = right_clearance
            self.latest_scan_geometry = {
                'ranges': tuple(message.ranges),
                'angle_min': float(message.angle_min),
                'angle_increment': float(message.angle_increment),
                'range_min': float(message.range_min),
                'range_max': float(message.range_max),
            }
            self.scan_monotonic = time.monotonic()

    def odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                float(orientation.w) * float(orientation.z)
                + float(orientation.x) * float(orientation.y)
            ),
            1.0 - 2.0 * (
                float(orientation.y) * float(orientation.y)
                + float(orientation.z) * float(orientation.z)
            ),
        )
        with self.data_lock:
            self.latest_odom = (float(position.x), float(position.y))
            self.latest_yaw = yaw
            self.odom_monotonic = time.monotonic()

    def snapshot(self):
        with self.data_lock:
            return (
                self.latest_scan,
                self.scan_monotonic,
                self.latest_odom,
                self.odom_monotonic,
            )

    def arc_snapshot(self, turn_direction):
        with self.data_lock:
            side_clearance = (
                self.latest_left_clearance
                if turn_direction > 0
                else self.latest_right_clearance
            )
            pose = None
            if self.latest_odom is not None and self.latest_yaw is not None:
                pose = (
                    self.latest_odom[0],
                    self.latest_odom[1],
                    self.latest_yaw,
                )
            return (
                self.latest_scan,
                side_clearance,
                self.latest_scan_geometry,
                self.scan_monotonic,
                pose,
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

    def arc_left_callback(self, _request, response):
        return self.arc_callback(response, turn_direction=1)

    def arc_right_callback(self, _request, response):
        return self.arc_callback(response, turn_direction=-1)

    def arc_callback(self, response, *, turn_direction):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A guarded escape is already active'
            return response
        try:
            return self.run_arc(response, turn_direction=turn_direction)
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

    def run_arc(self, response, *, turn_direction):
        direction_name = 'left' if turn_direction > 0 else 'right'
        now = time.monotonic()
        rear, side, scan, scan_time, start, odom_time = self.arc_snapshot(
            turn_direction,
        )
        if (
            rear is None
            or side is None
            or scan is None
            or now - scan_time > self.sensor_timeout
        ):
            response.success = False
            response.message = (
                f'Guarded {direction_name} arc refused: LaserScan is stale'
            )
            return response
        if start is None or now - odom_time > self.sensor_timeout:
            response.success = False
            response.message = (
                f'Guarded {direction_name} arc refused: '
                'wheel odometry is stale'
            )
            return response
        if rear < self.arc_start_rear_clearance:
            response.success = False
            response.message = (
                f'Guarded {direction_name} arc refused: rear clearance '
                f'{rear:.2f} m is below {self.arc_start_rear_clearance:.2f} m'
            )
            return response
        if side < self.arc_start_side_clearance:
            response.success = False
            response.message = (
                f'Guarded {direction_name} arc refused: '
                f'{direction_name} clearance '
                f'{side:.2f} m is below {self.arc_start_side_clearance:.2f} m'
            )
            return response
        obstacle = swept_arc_obstacle_distance(
            turn_direction=turn_direction,
            arc_speed=self.arc_speed,
            arc_turn_rate=self.arc_turn_rate,
            arc_yaw=self.arc_yaw,
            **scan,
        )
        if obstacle is not None:
            response.success = False
            response.message = (
                f'Guarded {direction_name} arc refused: an obstacle intersects '
                f'the swept footprint at {obstacle:.2f} m'
            )
            return response

        command = Twist()
        command.linear.x = -self.arc_speed
        command.angular.z = turn_direction * self.arc_turn_rate
        deadline = now + self.arc_time_limit
        while time.monotonic() < deadline:
            now = time.monotonic()
            rear, side, scan, scan_time, pose, odom_time = self.arc_snapshot(
                turn_direction,
            )
            if (
                rear is None
                or side is None
                or scan is None
                or now - scan_time > self.sensor_timeout
            ):
                response.success = False
                response.message = (
                    f'Guarded {direction_name} arc stopped: '
                    'LaserScan became stale'
                )
                return response
            obstacle = swept_arc_obstacle_distance(
                turn_direction=turn_direction,
                arc_speed=self.arc_speed,
                arc_turn_rate=self.arc_turn_rate,
                arc_yaw=self.arc_yaw,
                **scan,
            )
            if obstacle is not None:
                response.success = False
                response.message = (
                    f'Guarded {direction_name} arc stopped: an obstacle entered '
                    f'the swept footprint at {obstacle:.2f} m'
                )
                return response
            if rear < self.arc_stop_rear_clearance:
                response.success = False
                response.message = (
                    f'Guarded {direction_name} arc stopped: rear clearance '
                    f'fell to {rear:.2f} m'
                )
                return response
            if side < self.arc_stop_side_clearance:
                response.success = False
                response.message = (
                    f'Guarded {direction_name} arc stopped: {direction_name} '
                    f'clearance fell to {side:.2f} m'
                )
                return response
            if pose is None or now - odom_time > self.sensor_timeout:
                response.success = False
                response.message = (
                    f'Guarded {direction_name} arc stopped: '
                    'wheel odometry became stale'
                )
                return response

            travelled, directed_yaw = measured_arc_progress(
                start,
                pose,
                turn_direction,
            )
            if directed_yaw < -0.08:
                response.success = False
                response.message = (
                    f'Guarded {direction_name} arc stopped: '
                    'odometry reported rotation in the opposite direction'
                )
                return response
            if arc_motion_complete(
                start,
                pose,
                turn_direction=turn_direction,
                target_distance=self.arc_distance,
                target_yaw=self.arc_yaw,
            ):
                response.success = True
                response.message = (
                    f'Guarded {direction_name} arc completed '
                    f'{travelled:.2f} m / '
                    f'{math.degrees(directed_yaw):.1f} deg; '
                    f'rear {rear:.2f} m, side {side:.2f} m'
                )
                return response
            self.publisher.publish(command)
            time.sleep(0.05)

        response.success = False
        response.message = (
            f'Guarded {direction_name} arc timed out before measured '
            'distance and yaw completed'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GuardedReverse()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        if rclpy.ok():
            node.publish_stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
