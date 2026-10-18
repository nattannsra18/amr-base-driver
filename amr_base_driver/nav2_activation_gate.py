from __future__ import annotations

import ast
import math

from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


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
        self.declare_parameter(
            'manager_service', '/lifecycle_manager_navigation/manage_nodes')
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        service_name = self.get_parameter('manager_service').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = self.create_client(ManageLifecycleNodes, service_name)
        self.parameter_clients = {
            name: AsyncParameterClient(self, name)
            for name in EXPECTED_RUNTIME_PARAMETERS
        }
        self.request_in_flight = False
        self.navigation_started = False
        self.validation_pending = []
        self.runtime_values = {}
        self.timer = self.create_timer(0.5, self.check_localization)
        self.get_logger().info(
            'Waiting for an initial pose and '
            f'{self.global_frame} -> {self.base_frame} before Nav2 startup')

    def check_localization(self):
        if self.navigation_started or self.request_in_flight:
            return
        if not self.buffer.can_transform(
                self.global_frame, self.base_frame, Time()):
            return
        if not self.client.service_is_ready():
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.request_in_flight = True
        self.client.call_async(request).add_done_callback(self.startup_complete)
        self.get_logger().info(
            'Localization transform is available; requesting Nav2 startup')

    def startup_complete(self, future):
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().error(f'Nav2 startup service failed: {error}')
            return
        if not response.success:
            self.get_logger().error(
                'Nav2 lifecycle startup was rejected; retrying while '
                'localization remains available')
            return
        self.navigation_started = True
        self.timer.cancel()
        self.validation_pending = list(EXPECTED_RUNTIME_PARAMETERS)
        self.get_logger().info(
            'Nav2 lifecycle startup completed; validating runtime contract')
        self.request_next_parameters()

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
        rclpy.shutdown(context=self.context)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActivationGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
