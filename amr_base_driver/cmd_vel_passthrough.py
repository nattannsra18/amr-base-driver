import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class EscapePriorityGate:
    """Give a bounded recovery stream priority over normal velocity commands."""

    def __init__(self, lease_seconds=0.20):
        self.lease_seconds = float(lease_seconds)
        self.escape_until = 0.0
        self.escape_active = False

    def accept_normal(self, now):
        return not self.escape_active or now >= self.escape_until

    def accept_escape(self, now):
        self.escape_active = True
        self.escape_until = now + self.lease_seconds

    def expire(self, now):
        if self.escape_active and now >= self.escape_until:
            self.escape_active = False
            return True
        return False


class CmdVelPassthrough(Node):
    """Relay one command source at a time to the hardware serial bridge."""

    def __init__(self):
        super().__init__('cmd_vel_passthrough')
        self.declare_parameter('escape_lease_seconds', 0.20)
        self.gate = EscapePriorityGate(
            self.get_parameter('escape_lease_seconds').value)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.normal_command, 10)
        self.escape_subscription = self.create_subscription(
            Twist, '/cmd_vel_escape', self.escape_command, 10)
        self.watchdog = self.create_timer(0.05, self.expire_escape)

    def normal_command(self, command):
        if self.gate.accept_normal(time.monotonic()):
            self.publisher.publish(command)

    def escape_command(self, command):
        self.gate.accept_escape(time.monotonic())
        self.publisher.publish(command)

    def expire_escape(self):
        if self.gate.expire(time.monotonic()):
            # Never hand control back with the last reverse command latched.
            self.publish_stop()

    def publish_stop(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPassthrough()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
