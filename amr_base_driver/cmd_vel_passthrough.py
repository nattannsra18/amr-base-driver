from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CmdVelPassthrough(Node):
    """Relay commands only when the Drive Supervisor is explicitly disabled."""

    def __init__(self):
        super().__init__('cmd_vel_passthrough')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.publisher.publish, 10)

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
