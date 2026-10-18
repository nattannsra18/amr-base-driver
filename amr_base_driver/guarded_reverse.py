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
DEFAULT_WHEEL_SEPARATION = 0.36094
DEFAULT_ARC_YAW = 0.22
DEFAULT_ARC_DISTANCE = 0.5 * DEFAULT_WHEEL_SEPARATION * DEFAULT_ARC_YAW


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


def robust_front_clearance(
    ranges,
    *,
    angle_min,
    angle_increment,
    range_min,
    range_max,
):
    """Return the nearest adjacent-beam obstacle cluster ahead of the robot."""
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
        if abs(math.degrees(angle)) <= 30.0:
            values.append((index, distance))
    clearance = adjacent_cluster_clearance(values, beam_count=len(ranges))
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
    motion_direction=-1,
    arc_speed=0.07,
    arc_turn_rate=0.28,
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
    if motion_direction not in (-1, 1):
        raise ValueError('motion_direction must be -1 or 1')
    speed = max(0.001, abs(float(arc_speed)))
    turn_rate = max(0.001, abs(float(arc_turn_rate)))
    target_yaw = max(0.01, abs(float(arc_yaw)))
    angular_velocity = turn_direction * turn_rate
    linear_velocity = motion_direction * speed
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
        inside_physical_footprint = (
            FOOTPRINT_MIN_X <= point_x <= FOOTPRINT_MAX_X
            and FOOTPRINT_MIN_Y <= point_y <= FOOTPRINT_MAX_Y
        )
        inside_padded_footprint = (
            FOOTPRINT_MIN_X - FOOTPRINT_PADDING
            <= point_x
            <= FOOTPRINT_MAX_X + FOOTPRINT_PADDING
            and FOOTPRINT_MIN_Y - FOOTPRINT_PADDING
            <= point_y
            <= FOOTPRINT_MAX_Y + FOOTPRINT_PADDING
        )
        initial_padding_penetration = None
        if inside_padded_footprint and not inside_physical_footprint:
            initial_padding_penetration = min(
                point_x - (FOOTPRINT_MIN_X - FOOTPRINT_PADDING),
                FOOTPRINT_MAX_X + FOOTPRINT_PADDING - point_x,
                point_y - (FOOTPRINT_MIN_Y - FOOTPRINT_PADDING),
                FOOTPRINT_MAX_Y + FOOTPRINT_PADDING - point_y,
            )
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
                padding_penetration = min(
                    relative_x - (FOOTPRINT_MIN_X - FOOTPRINT_PADDING),
                    FOOTPRINT_MAX_X + FOOTPRINT_PADDING - relative_x,
                    relative_y - (FOOTPRINT_MIN_Y - FOOTPRINT_PADDING),
                    FOOTPRINT_MAX_Y + FOOTPRINT_PADDING - relative_y,
                )
                # A return may already overlap only the safety padding. Allow
                # an arc that continuously moves away from that return, while
                # still rejecting the opposite arc that moves any deeper.
                if (
                    initial_padding_penetration is not None
                    and padding_penetration
                    <= initial_padding_penetration + 1e-6
                ):
                    continue
                collisions.append((index, distance))
                break
    return adjacent_cluster_clearance(
        collisions,
        beam_count=len(ranges),
    )


def swept_straight_obstacle_distance(
    ranges,
    *,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    motion_direction,
    distance=0.10,
):
    """Return a clustered obstacle intersecting a straight footprint sweep."""
    if motion_direction not in (-1, 1):
        raise ValueError('motion_direction must be -1 or 1')
    target_distance = max(0.01, abs(float(distance)))
    steps = max(6, int(math.ceil(target_distance / 0.01)))
    poses_x = [
        motion_direction * target_distance * step / steps
        for step in range(1, steps + 1)
    ]
    collisions = []
    for index, raw_range in enumerate(ranges):
        measured_range = float(raw_range)
        if (
            not math.isfinite(measured_range)
            or not range_min <= measured_range <= range_max
        ):
            continue
        angle = angle_min + index * angle_increment
        point_x = LIDAR_X + measured_range * math.cos(angle)
        point_y = LIDAR_Y + measured_range * math.sin(angle)
        inside_current_footprint = (
            FOOTPRINT_MIN_X - FOOTPRINT_PADDING
            <= point_x
            <= FOOTPRINT_MAX_X + FOOTPRINT_PADDING
            and FOOTPRINT_MIN_Y - FOOTPRINT_PADDING
            <= point_y
            <= FOOTPRINT_MAX_Y + FOOTPRINT_PADDING
        )
        # A close point can already lie inside the padded footprint at the
        # opposite end of the chassis.  Moving away makes that overlap shrink;
        # it must not falsely block the escape direction.
        if inside_current_footprint and motion_direction * point_x < 0.0:
            continue
        for pose_x in poses_x:
            if (
                FOOTPRINT_MIN_X - FOOTPRINT_PADDING
                <= point_x - pose_x
                <= FOOTPRINT_MAX_X + FOOTPRINT_PADDING
                and FOOTPRINT_MIN_Y - FOOTPRINT_PADDING
                <= point_y
                <= FOOTPRINT_MAX_Y + FOOTPRINT_PADDING
            ):
                collisions.append((index, measured_range))
                break
    return adjacent_cluster_clearance(collisions, beam_count=len(ranges))


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


def rolling_pivot_turn_rate(speed, wheel_separation):
    """Return the yaw rate that leaves the inner wheel stationary."""
    return 2.0 * abs(float(speed)) / max(0.10, float(wheel_separation))


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
        self.declare_parameter('forward_start_clearance', 0.48)
        self.declare_parameter('forward_stop_clearance', 0.36)
        self.declare_parameter('sensor_timeout', 0.50)
        self.declare_parameter('time_limit', 3.0)
        self.declare_parameter('arc_speed', 0.07)
        self.declare_parameter('wheel_separation', DEFAULT_WHEEL_SEPARATION)
        # A rolling pivot follows a centre-line arc at half the wheel
        # separation.  Requiring 7 cm for 0.22 rad made the guard time out
        # after the requested rotation had already completed.
        self.declare_parameter('arc_distance', DEFAULT_ARC_DISTANCE)
        self.declare_parameter('arc_yaw', DEFAULT_ARC_YAW)
        self.declare_parameter('arc_start_rear_clearance', 0.32)
        self.declare_parameter('arc_stop_rear_clearance', 0.24)
        self.declare_parameter('arc_start_side_clearance', 0.40)
        self.declare_parameter('arc_stop_side_clearance', 0.28)
        self.declare_parameter('forward_arc_start_clearance', 0.48)
        self.declare_parameter('forward_arc_stop_clearance', 0.36)
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
        self.forward_start_clearance = max(0.45, float(
            self.get_parameter('forward_start_clearance').value))
        self.forward_stop_clearance = max(0.32, min(
            self.forward_start_clearance,
            float(self.get_parameter('forward_stop_clearance').value),
        ))
        self.sensor_timeout = min(1.0, max(0.20, float(
            self.get_parameter('sensor_timeout').value)))
        self.time_limit = min(4.0, max(1.0, float(
            self.get_parameter('time_limit').value)))
        self.arc_speed = min(0.07, max(0.03, float(
            self.get_parameter('arc_speed').value)))
        wheel_separation = max(0.10, float(
            self.get_parameter('wheel_separation').value))
        # Pivot around the stationary inner wheel.  A shallow arc requested
        # only ~5 RPM from that wheel, below this drivetrain's controllable
        # range, so minimum PWM made the robot turn in the opposite direction.
        self.arc_turn_rate = rolling_pivot_turn_rate(
            self.arc_speed, wheel_separation)
        self.arc_distance = min(0.14, max(0.03, float(
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
        self.forward_arc_start_clearance = max(0.45, float(
            self.get_parameter('forward_arc_start_clearance').value))
        self.forward_arc_stop_clearance = max(0.32, min(
            self.forward_arc_start_clearance,
            float(self.get_parameter('forward_arc_stop_clearance').value),
        ))
        self.arc_time_limit = min(5.0, max(1.5, float(
            self.get_parameter('arc_time_limit').value)))

        self.data_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.latest_scan_geometry = None
        self.scan_monotonic = 0.0
        self.latest_odom = None
        self.latest_yaw = None
        self.odom_monotonic = 0.0
        callbacks = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(Twist, '/cmd_vel_recovery', 10)
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
            '/guarded_forward_escape',
            self.forward_callback,
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
        self.create_service(
            Trigger,
            '/guarded_forward_arc_escape_left',
            self.forward_arc_left_callback,
            callback_group=callbacks,
        )
        self.create_service(
            Trigger,
            '/guarded_forward_arc_escape_right',
            self.forward_arc_right_callback,
            callback_group=callbacks,
        )

    def scan_callback(self, message):
        with self.data_lock:
            # Keep the immutable callback snapshot by reference.  Copying every
            # range at sensor rate consumed a full CPU core fraction even while
            # recovery was idle.  Geometry is materialized only when a recovery
            # service actually needs it.
            self.latest_scan_geometry = message
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

    def straight_snapshot(self, motion_direction):
        with self.data_lock:
            scan = self.latest_scan_geometry
            scan_time = self.scan_monotonic
            odom = self.latest_odom
            odom_time = self.odom_monotonic
        clearance = scan_geometry = None
        if scan is not None:
            scan_geometry = {
                'ranges': scan.ranges,
                'angle_min': float(scan.angle_min),
                'angle_increment': float(scan.angle_increment),
                'range_min': float(scan.range_min),
                'range_max': float(scan.range_max),
            }
            if motion_direction < 0:
                clearance = recovery_sector_clearances(**scan_geometry)[0]
            else:
                clearance = robust_front_clearance(**scan_geometry)
        return clearance, scan_geometry, scan_time, odom, odom_time

    def arc_snapshot(self, turn_direction, motion_direction=-1):
        with self.data_lock:
            scan = self.latest_scan_geometry
            scan_time = self.scan_monotonic
            pose = None
            if self.latest_odom is not None and self.latest_yaw is not None:
                pose = (
                    self.latest_odom[0],
                    self.latest_odom[1],
                    self.latest_yaw,
                )
            odom_time = self.odom_monotonic
        longitudinal = side_clearance = None
        if scan is not None:
            scan = {
                'ranges': scan.ranges,
                'angle_min': float(scan.angle_min),
                'angle_increment': float(scan.angle_increment),
                'range_min': float(scan.range_min),
                'range_max': float(scan.range_max),
            }
            rear, left, right = recovery_sector_clearances(**scan)
            longitudinal = rear if motion_direction < 0 else robust_front_clearance(
                **scan
            )
            side_clearance = left if turn_direction > 0 else right
        return (
            longitudinal,
            side_clearance,
            scan,
            scan_time,
            pose,
            odom_time,
        )

    def publish_stop(self):
        self.publisher.publish(Twist())

    def reverse_callback(self, _request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A guarded reverse is already active'
            return response
        try:
            return self.run_straight(response, motion_direction=-1)
        finally:
            for _ in range(5):
                self.publish_stop()
                time.sleep(0.05)
            self.operation_lock.release()

    def forward_callback(self, _request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A guarded escape is already active'
            return response
        try:
            return self.run_straight(response, motion_direction=1)
        finally:
            for _ in range(5):
                self.publish_stop()
                time.sleep(0.05)
            self.operation_lock.release()

    def arc_left_callback(self, _request, response):
        return self.arc_callback(response, turn_direction=1, motion_direction=-1)

    def arc_right_callback(self, _request, response):
        return self.arc_callback(response, turn_direction=-1, motion_direction=-1)

    def forward_arc_left_callback(self, _request, response):
        return self.arc_callback(response, turn_direction=1, motion_direction=1)

    def forward_arc_right_callback(self, _request, response):
        return self.arc_callback(response, turn_direction=-1, motion_direction=1)

    def arc_callback(self, response, *, turn_direction, motion_direction):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A guarded escape is already active'
            return response
        try:
            return self.run_arc(
                response,
                turn_direction=turn_direction,
                motion_direction=motion_direction,
            )
        finally:
            for _ in range(5):
                self.publish_stop()
                time.sleep(0.05)
            self.operation_lock.release()

    def run_straight(self, response, *, motion_direction):
        motion_name = 'reverse' if motion_direction < 0 else 'forward'
        longitudinal_name = 'rear' if motion_direction < 0 else 'front'
        start_clearance = (
            self.start_clearance
            if motion_direction < 0
            else self.forward_start_clearance
        )
        stop_clearance = (
            self.stop_clearance
            if motion_direction < 0
            else self.forward_stop_clearance
        )
        now = time.monotonic()
        clearance, scan, scan_time, start, odom_time = (
            self.straight_snapshot(motion_direction)
        )
        label = f'Guarded {motion_name}'
        if (
            clearance is None
            or scan is None
            or now - scan_time > self.sensor_timeout
        ):
            response.success = False
            response.message = f'{label} refused: LaserScan is stale'
            return response
        if start is None or now - odom_time > self.sensor_timeout:
            response.success = False
            response.message = f'{label} refused: wheel odometry is stale'
            return response
        if clearance < start_clearance:
            response.success = False
            response.message = (
                f'{label} refused: {longitudinal_name} clearance '
                f'{clearance:.2f} m is below {start_clearance:.2f} m'
            )
            return response
        obstacle = swept_straight_obstacle_distance(
            motion_direction=motion_direction,
            distance=self.distance,
            **scan,
        )
        if obstacle is not None:
            response.success = False
            response.message = (
                f'{label} refused: an obstacle intersects the swept footprint '
                f'at {obstacle:.2f} m'
            )
            return response

        command = Twist()
        command.linear.x = motion_direction * self.speed
        deadline = now + self.time_limit
        while time.monotonic() < deadline:
            now = time.monotonic()
            clearance, scan, scan_time, position, odom_time = (
                self.straight_snapshot(motion_direction)
            )
            if (
                clearance is None
                or scan is None
                or now - scan_time > self.sensor_timeout
                or clearance < stop_clearance
            ):
                response.success = False
                response.message = (
                    f'{label} stopped: {longitudinal_name} path is no longer safe'
                )
                return response
            obstacle = swept_straight_obstacle_distance(
                motion_direction=motion_direction,
                distance=self.distance,
                **scan,
            )
            if obstacle is not None:
                response.success = False
                response.message = (
                    f'{label} stopped: an obstacle entered the swept footprint '
                    f'at {obstacle:.2f} m'
                )
                return response
            if position is None or now - odom_time > self.sensor_timeout:
                response.success = False
                response.message = f'{label} stopped: wheel odometry became stale'
                return response
            travelled = math.hypot(position[0] - start[0], position[1] - start[1])
            if travelled >= self.distance:
                response.success = True
                response.message = (
                    f'{label} completed {travelled:.2f} m; '
                    f'{longitudinal_name} clearance {clearance:.2f} m'
                )
                return response
            self.publisher.publish(command)
            time.sleep(0.05)

        response.success = False
        response.message = (
            f'{label} timed out before measured motion completed'
        )
        return response

    def run_arc(self, response, *, turn_direction, motion_direction=-1):
        direction_name = 'left' if turn_direction > 0 else 'right'
        motion_name = 'reverse' if motion_direction < 0 else 'forward'
        longitudinal_name = 'rear' if motion_direction < 0 else 'front'
        start_clearance = (
            self.arc_start_rear_clearance
            if motion_direction < 0
            else self.forward_arc_start_clearance
        )
        stop_clearance = (
            self.arc_stop_rear_clearance
            if motion_direction < 0
            else self.forward_arc_stop_clearance
        )
        now = time.monotonic()
        longitudinal, side, scan, scan_time, start, odom_time = (
            self.arc_snapshot(turn_direction, motion_direction)
        )
        label = f'Guarded {motion_name} {direction_name} arc'
        if (
            longitudinal is None
            or side is None
            or scan is None
            or now - scan_time > self.sensor_timeout
        ):
            response.success = False
            response.message = f'{label} refused: LaserScan is stale'
            return response
        if start is None or now - odom_time > self.sensor_timeout:
            response.success = False
            response.message = f'{label} refused: wheel odometry is stale'
            return response
        if longitudinal < start_clearance:
            response.success = False
            response.message = (
                f'{label} refused: {longitudinal_name} clearance '
                f'{longitudinal:.2f} m is below {start_clearance:.2f} m'
            )
            return response
        if side < self.arc_start_side_clearance:
            response.success = False
            response.message = (
                f'{label} refused: {direction_name} clearance '
                f'{side:.2f} m is below {self.arc_start_side_clearance:.2f} m'
            )
            return response
        obstacle = swept_arc_obstacle_distance(
            turn_direction=turn_direction,
            motion_direction=motion_direction,
            arc_speed=self.arc_speed,
            arc_turn_rate=self.arc_turn_rate,
            arc_yaw=self.arc_yaw,
            **scan,
        )
        if obstacle is not None:
            response.success = False
            response.message = (
                f'{label} refused: an obstacle intersects the swept footprint '
                f'at {obstacle:.2f} m'
            )
            return response

        command = Twist()
        command.linear.x = motion_direction * self.arc_speed
        command.angular.z = turn_direction * self.arc_turn_rate
        deadline = now + self.arc_time_limit
        while time.monotonic() < deadline:
            now = time.monotonic()
            longitudinal, side, scan, scan_time, pose, odom_time = (
                self.arc_snapshot(turn_direction, motion_direction)
            )
            if (
                longitudinal is None
                or side is None
                or scan is None
                or now - scan_time > self.sensor_timeout
            ):
                response.success = False
                response.message = f'{label} stopped: LaserScan became stale'
                return response
            obstacle = swept_arc_obstacle_distance(
                turn_direction=turn_direction,
                motion_direction=motion_direction,
                arc_speed=self.arc_speed,
                arc_turn_rate=self.arc_turn_rate,
                arc_yaw=self.arc_yaw,
                **scan,
            )
            if obstacle is not None:
                response.success = False
                response.message = (
                    f'{label} stopped: an obstacle entered the swept footprint '
                    f'at {obstacle:.2f} m'
                )
                return response
            if longitudinal < stop_clearance:
                response.success = False
                response.message = (
                    f'{label} stopped: {longitudinal_name} clearance fell to '
                    f'{longitudinal:.2f} m'
                )
                return response
            if side < self.arc_stop_side_clearance:
                response.success = False
                response.message = (
                    f'{label} stopped: {direction_name} clearance fell to '
                    f'{side:.2f} m'
                )
                return response
            if pose is None or now - odom_time > self.sensor_timeout:
                response.success = False
                response.message = f'{label} stopped: wheel odometry became stale'
                return response

            travelled, directed_yaw = measured_arc_progress(
                start,
                pose,
                turn_direction,
            )
            if directed_yaw < -0.08:
                response.success = False
                response.message = (
                    f'{label} stopped: odometry reported rotation in the '
                    'opposite direction'
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
                    f'{label} completed {travelled:.2f} m / '
                    f'{math.degrees(directed_yaw):.1f} deg; '
                    f'{longitudinal_name} {longitudinal:.2f} m, '
                    f'side {side:.2f} m'
                )
                return response
            self.publisher.publish(command)
            time.sleep(0.05)

        response.success = False
        response.message = (
            f'{label} timed out before measured distance and yaw completed'
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
