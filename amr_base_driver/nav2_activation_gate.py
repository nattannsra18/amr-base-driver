from __future__ import annotations

import ast
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .lifecycle_interlock import LifecycleInterlock


EXPECTED_RUNTIME_PARAMETERS = {
    '/local_costmap/local_costmap': {
        'global_frame': 'odom', 'robot_base_frame': 'base_footprint',
        'rolling_window': True, 'width': 3.0, 'height': 3.0,
        'resolution': 0.05, 'footprint_padding': 0.02,
        'plugins': ['obstacle_layer', 'inflation_layer'],
    },
    '/global_costmap/global_costmap': {
        'global_frame': 'map', 'robot_base_frame': 'base_footprint',
        'resolution': 0.03, 'footprint_padding': 0.02,
        'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
    },
    '/controller_server': {
        'controller_plugins': ['FollowPath'],
        'odom_topic': '/odometry/filtered',
        'FollowPath.use_collision_detection': True,
        'FollowPath.use_rotate_to_heading': True,
        'FollowPath.allow_reversing': False,
    },
    '/collision_monitor': {
        'base_frame_id': 'base_footprint', 'odom_frame_id': 'odom',
        'cmd_vel_in_topic': '/cmd_vel_smoothed',
        'cmd_vel_out_topic': '/cmd_vel_navigation',
        'polygons': ['FootprintApproach'],
        'FootprintApproach.footprint_topic': '/local_costmap/published_footprint',
        'observation_sources': ['scan'], 'scan.topic': '/scan',
    },
}
EXPECTED_RUNTIME_PUBLISHERS = (
    '/odometry/filtered',
    '/scan',
)
EXPECTED_FOOTPRINT = [
    [0.303, 0.190], [0.303, -0.190],
    [-0.087, -0.190], [-0.087, 0.190],
]


def _equivalent(actual, expected):
    if isinstance(expected, float):
        try:
            return math.isclose(float(actual), expected, abs_tol=1e-6)
        except (TypeError, ValueError):
            return False
    return actual == expected


def validate_runtime_parameters(values):
    """Return all physical-profile contract violations."""
    errors = []
    for node_name, expected in EXPECTED_RUNTIME_PARAMETERS.items():
        actual = values.get(node_name, {})
        for parameter_name, expected_value in expected.items():
            actual_value = actual.get(parameter_name)
            if not _equivalent(actual_value, expected_value):
                errors.append(
                    f'{node_name}.{parameter_name}={actual_value!r}; '
                    f'expected {expected_value!r}'
                )
        if node_name.endswith('costmap'):
            footprint = actual.get('footprint')
            try:
                parsed = ast.literal_eval(footprint) if isinstance(
                    footprint, str
                ) else footprint
            except (SyntaxError, ValueError):
                parsed = None
            if parsed != EXPECTED_FOOTPRINT:
                errors.append(
                    f'{node_name}.footprint={footprint!r}; expected physical '
                    f'footprint {EXPECTED_FOOTPRINT!r}'
                )
    return errors


class Nav2ActivationGate(Node):
    """Start Nav2 after localization, then fail safe on config drift."""

    def __init__(self):
        super().__init__('nav2_activation_gate')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('max_amcl_pose_age_seconds', 2.0)
        self.declare_parameter('max_transform_age_seconds', 1.0)
        self.declare_parameter('max_transform_future_seconds', 1.25)
        self.declare_parameter(
            'manager_service', '/lifecycle_manager_navigation/manage_nodes')
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.max_amcl_pose_age_seconds = float(
            self.get_parameter('max_amcl_pose_age_seconds').value)
        self.max_transform_age_seconds = float(
            self.get_parameter('max_transform_age_seconds').value)
        self.max_transform_future_seconds = float(
            self.get_parameter('max_transform_future_seconds').value)
        service_name = self.get_parameter('manager_service').value
        self.buffer = Buffer()
        # /tf is one of the busiest topics on the robot.  Do not deserialize
        # it for minutes while waiting for the operator to set an initial
        # pose; AMCL cannot provide useful localization evidence before then.
        self.listener = None
        self.latest_amcl_pose_stamp = None
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_pose_topic,
            self.amcl_pose_received,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.client = self.create_client(ManageLifecycleNodes, service_name)
        self.planner_state_client = self.create_client(
            GetState, '/planner_server/get_state')
        # Parameter clients create several DDS endpoints each.  Nav2 cannot be
        # validated until localization is fresh, so create them only once the
        # lifecycle startup has actually completed.
        self.parameter_clients = {}
        self.request_in_flight = False
        self.navigation_started = False
        self.lifecycle_interlock = LifecycleInterlock()
        self.retry_delay_seconds = 0.5
        self.next_retry_monotonic = 0.0
        self.lifecycle_repair_attempted = False
        self.validation_pending = []
        self.runtime_values = {}
        self.timer = self.create_timer(1.0, self.check_localization)
        self.get_logger().info(
            'Waiting for an initial pose and '
            f'{self.global_frame} -> {self.base_frame} before Nav2 startup')

    def amcl_pose_received(self, message: PoseWithCovarianceStamped) -> None:
        """Record only AMCL evidence received during this gate session."""
        self.latest_amcl_pose_stamp = Time.from_msg(message.header.stamp)
        if self.listener is None:
            self.listener = TransformListener(self.buffer, self)

    @staticmethod
    def timestamp_age_seconds(now: Time, stamp: Time) -> float:
        return (now - stamp).nanoseconds / 1_000_000_000.0

    def localization_is_fresh(self) -> bool:
        """Require a current-session AMCL pose and a fresh map transform."""
        if self.latest_amcl_pose_stamp is None:
            return False
        now = self.get_clock().now()
        pose_age = self.timestamp_age_seconds(
            now, self.latest_amcl_pose_stamp)
        if pose_age < 0.0 or pose_age > self.max_amcl_pose_age_seconds:
            return False
        try:
            transform = self.buffer.lookup_transform(
                self.global_frame, self.base_frame, Time())
        except TransformException:
            return False
        transform_stamp = Time.from_msg(transform.header.stamp)
        transform_age = self.timestamp_age_seconds(now, transform_stamp)
        # AMCL intentionally future-dates map -> odom by transform_tolerance.
        return (
            -self.max_transform_future_seconds
            <= transform_age
            <= self.max_transform_age_seconds
        )

    @staticmethod
    def mapping_active():
        return Path(
            '/var/lib/indoor-delivery-robot/mapping-active'
        ).is_file()

    def check_localization(self):
        if self.mapping_active():
            return
        if self.navigation_started or self.request_in_flight:
            return
        if time.monotonic() < self.next_retry_monotonic:
            return
        if not self.localization_is_fresh():
            return
        if not self.client.service_is_ready():
            return
        if not self.planner_state_client.service_is_ready():
            return
        # The marker is an immediate intent signal.  The advisory lock closes
        # the remaining check/call race and is held until the asynchronous
        # lifecycle request and runtime validation have fully completed.
        if not self.lifecycle_interlock.try_acquire():
            return
        if self.mapping_active():
            self.lifecycle_interlock.release()
            return
        self.request_in_flight = True
        try:
            self.planner_state_client.call_async(
                GetState.Request()
            ).add_done_callback(self.planner_state_complete)
        except Exception as error:
            self.fail_request(f'Nav2 state request failed: {error}')

    def planner_state_complete(self, future):
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.fail_request(f'Nav2 state request failed: {error}')
            return
        if self.mapping_active():
            self.request_in_flight = False
            self.lifecycle_interlock.release()
            return
        if int(response.current_state.id) == 3:
            self.request_in_flight = False
            self.begin_validation(
                'Nav2 is already active; validating runtime contract')
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        try:
            self.client.call_async(request).add_done_callback(
                self.startup_complete)
        except Exception as error:
            self.fail_request(f'Nav2 startup service failed: {error}')
            return
        self.get_logger().info(
            'Current-session AMCL pose and fresh localization transform are '
            'available; requesting Nav2 startup')

    def fail_request(self, message: str) -> None:
        self.request_in_flight = False
        self.get_logger().error(message)
        self.defer_retry()
        self.lifecycle_interlock.release()

    def startup_complete(self, future):
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.fail_request(f'Nav2 startup service failed: {error}')
            return
        if self.mapping_active():
            # Mapping announced ownership while STARTUP was already in
            # flight.  Do not validate or retry here; MappingRuntime will
            # acquire the lock next and pause whichever nodes became active.
            self.get_logger().info(
                'Mapping requested during Nav2 startup; deferring activation')
            self.lifecycle_interlock.release()
            return
        if not response.success:
            if not self.lifecycle_repair_attempted:
                self.lifecycle_repair_attempted = True
                self.request_lifecycle_reset()
                return
            self.stop_after_failed_repair(
                'Nav2 lifecycle startup was rejected after one reset; '
                'activation is stopping fail-closed')
            return
        self.retry_delay_seconds = 0.5
        self.next_retry_monotonic = 0.0
        self.begin_validation(
            'Nav2 lifecycle startup completed; validating runtime contract')

    def request_lifecycle_reset(self) -> None:
        """Repair one partial lifecycle startup, then retry from clean state."""
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.RESET
        self.request_in_flight = True
        try:
            self.client.call_async(request).add_done_callback(
                self.lifecycle_reset_complete)
        except Exception as error:
            self.stop_after_failed_repair(
                f'Nav2 lifecycle reset service failed: {error}')
            return
        self.get_logger().warning(
            'Nav2 lifecycle startup was rejected; resetting the managed '
            'nodes once before retrying')

    def lifecycle_reset_complete(self, future) -> None:
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.stop_after_failed_repair(
                f'Nav2 lifecycle reset service failed: {error}')
            return
        if not response.success:
            self.stop_after_failed_repair(
                'Nav2 lifecycle reset was rejected; activation is stopping '
                'fail-closed')
            return
        self.get_logger().info(
            'Nav2 lifecycle reset completed; retrying startup once')
        self.defer_retry()
        self.lifecycle_interlock.release()

    def stop_after_failed_repair(self, message: str) -> None:
        self.request_in_flight = False
        self.get_logger().fatal(message)
        self.timer.cancel()
        self.lifecycle_interlock.release()
        rclpy.shutdown(context=self.context)

    def begin_validation(self, message: str) -> None:
        self.retry_delay_seconds = 0.5
        self.next_retry_monotonic = 0.0
        self.navigation_started = True
        self.timer.cancel()
        self.parameter_clients = {
            name: AsyncParameterClient(self, name)
            for name in EXPECTED_RUNTIME_PARAMETERS
        }
        self.validation_pending = list(EXPECTED_RUNTIME_PARAMETERS)
        self.get_logger().info(message)
        self.request_next_parameters()

    def defer_retry(self) -> None:
        self.next_retry_monotonic = (
            time.monotonic() + self.retry_delay_seconds
        )
        self.retry_delay_seconds = min(
            5.0,
            max(0.5, self.retry_delay_seconds * 2.0),
        )

    def request_next_parameters(self):
        if not self.validation_pending:
            self.finish_validation()
            return
        node_name = self.validation_pending.pop(0)
        client = self.parameter_clients[node_name]
        if not client.services_are_ready():
            self.validation_pending.insert(0, node_name)
            self.timer = self.create_timer(0.25, self.retry_parameter_request)
            return
        names = list(EXPECTED_RUNTIME_PARAMETERS[node_name])
        if node_name.endswith('costmap'):
            names.append('footprint')
        client.get_parameters(names).add_done_callback(
            lambda result, node=node_name, requested=names: (
                self.parameters_received(node, requested, result)))

    def retry_parameter_request(self):
        self.timer.cancel()
        self.request_next_parameters()

    def parameters_received(self, node_name, names, future):
        try:
            response = future.result()
            self.runtime_values[node_name] = {
                name: parameter_value_to_python(value)
                for name, value in zip(names, response.values, strict=True)
            }
        except Exception as error:  # pragma: no cover - middleware failure
            self.runtime_values[node_name] = {'parameter_read_error': str(error)}
        self.request_next_parameters()

    def finish_validation(self):
        errors = validate_runtime_parameters(self.runtime_values)
        errors.extend(
            f'{topic} has no publisher'
            for topic in EXPECTED_RUNTIME_PUBLISHERS
            if self.count_publishers(topic) < 1
        )
        if not errors:
            self.get_logger().info(
                'Nav2 runtime contract verified; activation gate is exiting')
            self.lifecycle_interlock.release()
            rclpy.shutdown(context=self.context)
            return
        self.get_logger().fatal(
            'Nav2 runtime contract failed; pausing navigation: '
            + ' | '.join(errors))
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.PAUSE
        self.client.call_async(request).add_done_callback(
            lambda future: Nav2ActivationGate.pause_complete(self, future))

    def pause_complete(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().fatal('Nav2 lifecycle pause was rejected')
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().fatal(f'Nav2 lifecycle pause failed: {error}')
        self.lifecycle_interlock.release()
        rclpy.shutdown(context=self.context)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActivationGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.lifecycle_interlock.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
