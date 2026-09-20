from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class Nav2ActivationGate(Node):
    """Start navigation only after localization publishes map -> base."""

    def __init__(self):
        super().__init__('nav2_activation_gate')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter(
            'manager_service',
            '/lifecycle_manager_navigation/manage_nodes')
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        service_name = self.get_parameter('manager_service').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = self.create_client(ManageLifecycleNodes, service_name)
        self.request_in_flight = False
        self.navigation_started = False
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
        future = self.client.call_async(request)
        future.add_done_callback(self.startup_complete)
        self.get_logger().info(
            'Localization transform is available; requesting Nav2 startup')

    def startup_complete(self, future):
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().error(f'Nav2 startup service failed: {error}')
            return
        if response.success:
            self.navigation_started = True
            self.get_logger().info('Nav2 lifecycle startup completed')
        else:
            self.get_logger().error(
                'Nav2 lifecycle startup was rejected; retrying while '
                'localization remains available')


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
