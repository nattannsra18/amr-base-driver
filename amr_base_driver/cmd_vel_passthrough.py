import enum
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CommandOwner(str, enum.Enum):
    STOPPED = 'STOPPED'
    NAVIGATION = 'NAVIGATION'
    MANUAL = 'MANUAL'
    RECOVERY = 'RECOVERY'


class CommandArbiter:
    """Small lease-based state machine for exclusive base command ownership."""

    PRIORITY = (
        CommandOwner.STOPPED,
        CommandOwner.RECOVERY,
        CommandOwner.MANUAL,
        CommandOwner.NAVIGATION,
    )

    def __init__(self, lease_seconds=0.25):
        self.lease_seconds = float(lease_seconds)
        self.deadlines = {owner: 0.0 for owner in self.PRIORITY}
        self.owner = CommandOwner.STOPPED

    def accept(self, owner, now):
        if owner not in self.deadlines:
            raise ValueError(f'{owner} cannot publish motion commands')
        if (
            owner != CommandOwner.STOPPED
            and self.deadlines[CommandOwner.STOPPED] > now
        ):
            return False, False
        if owner == CommandOwner.STOPPED:
            for source in self.deadlines:
                self.deadlines[source] = 0.0
        self.deadlines[owner] = float(now) + self.lease_seconds
        previous = self.owner
        self.owner = self._select(now)
        return self.owner == owner, self.owner != previous

    def expire(self, now):
        previous = self.owner
        self.owner = self._select(now)
        return self.owner != previous

    def _select(self, now):
        for owner in self.PRIORITY:
            if self.deadlines[owner] > now:
                return owner
        return CommandOwner.STOPPED


class CmdVelPassthrough(Node):
    """Arbitrate NAVIGATION, RECOVERY and MANUAL into one safe motor stream."""

    def __init__(self):
        super().__init__('cmd_vel_passthrough')
        self.declare_parameter('command_lease_seconds', 0.25)
        self.arbiter = CommandArbiter(
            self.get_parameter('command_lease_seconds').value)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.diagnostics = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(
            Twist, '/cmd_vel_navigation',
            lambda command: self.command(CommandOwner.NAVIGATION, command), 10)
        self.create_subscription(
            Twist, '/cmd_vel_recovery',
            lambda command: self.command(CommandOwner.RECOVERY, command), 10)
        self.create_subscription(
            Twist, '/cmd_vel_manual',
            lambda command: self.command(CommandOwner.MANUAL, command), 10)
        self.create_subscription(
            Twist, '/cmd_vel_stop',
            lambda command: self.command(CommandOwner.STOPPED, command), 10)
        self.watchdog = self.create_timer(0.05, self.expire_owner)
        self.diagnostic_timer = self.create_timer(0.5, self.publish_diagnostic)
        self.get_logger().info(
            'Command ownership ready: NAVIGATION, RECOVERY, MANUAL -> '
            '/cmd_vel_safe')

    def command(self, owner, command):
        accepted, changed = self.arbiter.accept(owner, time.monotonic())
        if changed:
            # Ensure a command from the old owner cannot remain latched.
            self.publish_stop()
        if accepted and owner != CommandOwner.STOPPED:
            self.publisher.publish(command)
        elif accepted:
            self.publish_stop()

    def expire_owner(self):
        if self.arbiter.expire(time.monotonic()):
            self.publish_stop()

    def publish_diagnostic(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'Base command ownership'
        status.hardware_id = 'host-command-arbiter'
        status.level = DiagnosticStatus.OK
        status.message = self.arbiter.owner.value
        status.values = [KeyValue(key='owner', value=self.arbiter.owner.value)]
        message.status = [status]
        self.diagnostics.publish(message)

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
