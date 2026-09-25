"""Bring up localization before allowing the heavier Nav2 process group."""

import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.node import Node


class LocalizationReadinessGate(Node):
    """Retry localization startup and exit only after both nodes are active."""

    def __init__(self):
        super().__init__('localization_readiness_gate')
        self.declare_parameter('request_startup', True)
        self.should_request_startup = bool(
            self.get_parameter('request_startup').value)
        self.state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in ('map_server', 'amcl')
        }
        self.manager = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_localization/manage_nodes',
        )
        self.states = {name: None for name in self.state_clients}
        self.state_requests = 0
        self.startup_pending = False
        self.next_startup_attempt = 0.0
        self.ready = False
        self.timer = self.create_timer(0.5, self.check)
        self.get_logger().info(
            'Waiting for map_server and AMCL lifecycle readiness')

    def check(self):
        if self.ready:
            self.get_logger().info(
                'Localization is active; releasing Nav2 process startup')
            rclpy.shutdown()
            return
        if self.state_requests or self.startup_pending:
            return
        if not all(client.service_is_ready()
                   for client in self.state_clients.values()):
            return

        self.state_requests = len(self.state_clients)
        for name, client in self.state_clients.items():
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda result, node_name=name: self.state_received(
                    node_name, result))

    def state_received(self, name, future):
        try:
            self.states[name] = int(future.result().current_state.id)
        except Exception as error:  # pragma: no cover - middleware failure
            self.states[name] = None
            self.get_logger().warning(
                f'Unable to read {name} lifecycle state: {error}')
        self.state_requests -= 1
        if self.state_requests:
            return
        if all(state == State.PRIMARY_STATE_ACTIVE
               for state in self.states.values()):
            self.ready = True
            return
        if self.should_request_startup:
            self.request_startup()

    def request_startup(self):
        now = time.monotonic()
        if now < self.next_startup_attempt or not self.manager.service_is_ready():
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.startup_pending = True
        future = self.manager.call_async(request)
        future.add_done_callback(self.startup_complete)
        self.get_logger().info(
            'Localization is not active; requesting lifecycle startup')

    def startup_complete(self, future):
        self.startup_pending = False
        self.next_startup_attempt = time.monotonic() + 2.0
        try:
            success = bool(future.result().success)
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().warning(
                f'Localization startup service failed: {error}')
            return
        if not success:
            self.get_logger().warning(
                'Localization startup was rejected; readiness will retry')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationReadinessGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
