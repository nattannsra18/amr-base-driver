import math
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


HELP = '''
Game-style safe keyboard control (press or hold a key)
------------------------------------------------------
 Q: forward-left   W: forward      E: forward-right
 A: rotate left                     D: rotate right
 Z: reverse-left   S: reverse      C: reverse-right

1/2/3: slow / normal / fast speed profile
, / .: decrease / increase drive speed by 0.01 m/s
[ / ]: decrease / increase pivot speed by 0.05 rad/s
Space/X: STOP immediately
R: reset dashboard distance only (preserves SLAM odometry)
Esc or Ctrl-C: STOP and quit

Releasing the key automatically commands zero after the deadman timeout.
Keep the physical motor cutoff within reach.
'''


SPEED_PROFILES = {
    '1': (0.04, 0.25, 'SLOW'),
    '2': (0.08, 0.45, 'NORMAL'),
    '3': (0.14, 0.70, 'FAST'),
}

MAPPING_SPEED_PROFILES = {
    # Keep both wheels above the measured loaded-floor stiction region.
    '1': (0.08, 0.12, 'MAPPING SLOW'),
    '2': (0.10, 0.15, 'MAPPING NORMAL'),
    '3': (0.12, 0.18, 'MAPPING FAST'),
}

# GNOME's default key-repeat delay is 500 ms.  Keep the terminal deadman
# longer than that first gap so a held key does not stop once before repeat
# events begin.  Space/X still stop immediately, and the downstream command
# and firmware watchdogs remain active.
DEFAULT_KEY_TIMEOUT = 0.65
MAPPING_PIVOT_ANGULAR_SPEED = 0.60
MIN_LINEAR_SPEED = 0.02
MAX_LINEAR_SPEED = 0.20
LINEAR_SPEED_STEP = 0.01
MIN_PIVOT_SPEED = 0.15
MAX_PIVOT_SPEED = 1.00
PIVOT_SPEED_STEP = 0.05


def adjusted_speed(value, delta, minimum, maximum):
    """Apply one operator adjustment while retaining explicit safe bounds."""
    return min(max(value + delta, minimum), maximum)


def mapping_limits(linear, angular, enabled):
    """Limit mapping scan distortion; never modify ordinary driving profiles."""
    if not enabled:
        return linear, angular
    if linear:
        linear = math.copysign(
            min(max(abs(linear), 0.08), 0.12), linear)
    return linear, min(max(angular, -0.18), 0.18)


def pivot_angular_speed(angular, mapping_enabled):
    """Keep an in-place turn above the passive-caster stiction region."""
    if not mapping_enabled:
        return angular
    return max(abs(angular), MAPPING_PIVOT_ANGULAR_SPEED)


class SafeKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('safe_keyboard_teleop')
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('key_timeout', DEFAULT_KEY_TIMEOUT)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('mapping_mode', False)
        self.mapping_mode = bool(self.get_parameter('mapping_mode').value)
        self.linear_speed = float(
            self.get_parameter('linear_speed').value)
        self.angular_speed = float(
            self.get_parameter('angular_speed').value)
        self.linear_speed, self.angular_speed = mapping_limits(
            self.linear_speed, self.angular_speed, self.mapping_mode)
        self.pivot_speed = pivot_angular_speed(
            self.angular_speed, self.mapping_mode)
        self.key_timeout = float(self.get_parameter('key_timeout').value)
        self.publish_rate = float(
            self.get_parameter('publish_rate').value)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dashboard_reset = self.create_client(
            Empty, '/reset_dashboard_distance')
        self.linear = 0.0
        self.angular = 0.0
        self.last_motion_key = 0.0
        self.last_status = None

    def set_key(self, key):
        key = key.lower()
        if key == 'w':
            self.linear, self.angular = self.linear_speed, 0.0
        elif key == 's':
            self.linear, self.angular = -self.linear_speed, 0.0
        elif key == 'a':
            self.linear, self.angular = 0.0, self.pivot_speed
        elif key == 'd':
            self.linear, self.angular = 0.0, -self.pivot_speed
        elif key == 'q':
            self.linear, self.angular = (
                self.linear_speed, self.angular_speed)
        elif key == 'e':
            self.linear, self.angular = (
                self.linear_speed, -self.angular_speed)
        elif key == 'z':
            self.linear, self.angular = (
                -self.linear_speed, self.angular_speed)
        elif key == 'c':
            self.linear, self.angular = (
                -self.linear_speed, -self.angular_speed)
        elif key in SPEED_PROFILES:
            profiles = (MAPPING_SPEED_PROFILES if self.mapping_mode
                        else SPEED_PROFILES)
            linear, angular, name = profiles[key]
            linear, angular = mapping_limits(linear, angular, self.mapping_mode)
            self.linear_speed = linear
            self.angular_speed = angular
            self.pivot_speed = pivot_angular_speed(
                angular, self.mapping_mode)
            self.stop(f'{name} SPEED: {linear:.2f} m/s, '
                      f'curve {angular:.2f}, pivot '
                      f'{self.pivot_speed:.2f} rad/s')
            return True
        elif key in (',', '.'):
            delta = -LINEAR_SPEED_STEP if key == ',' else LINEAR_SPEED_STEP
            self.linear_speed = adjusted_speed(
                self.linear_speed, delta,
                MIN_LINEAR_SPEED, MAX_LINEAR_SPEED)
            self.stop(
                f'DRIVE SPEED SET TO {self.linear_speed:.2f} m/s')
            return True
        elif key in ('[', ']'):
            delta = -PIVOT_SPEED_STEP if key == '[' else PIVOT_SPEED_STEP
            self.pivot_speed = adjusted_speed(
                self.pivot_speed, delta,
                MIN_PIVOT_SPEED, MAX_PIVOT_SPEED)
            self.stop(
                f'PIVOT SPEED SET TO {self.pivot_speed:.2f} rad/s')
            return True
        elif key in (' ', 'x'):
            self.stop('STOP')
            return True
        elif key == 'r':
            if self.linear or self.angular:
                self.stop('STOP before distance reset')
            if self.dashboard_reset.service_is_ready():
                self.dashboard_reset.call_async(Empty.Request())
            self.show_status('DISTANCE RESET REQUESTED')
            return True
        else:
            return False
        self.last_motion_key = time.monotonic()
        return True

    def stop(self, reason='STOP'):
        self.linear = 0.0
        self.angular = 0.0
        self.last_motion_key = 0.0
        self.publish()
        self.show_status(reason)

    def expire_command(self):
        if (self.last_motion_key and
                time.monotonic() - self.last_motion_key > self.key_timeout):
            self.stop('AUTO STOP: release/keyboard timeout')

    def publish(self):
        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.publisher.publish(message)

    def show_status(self, reason=None):
        status = (round(self.linear, 3), round(self.angular, 3), reason)
        if status == self.last_status:
            return
        self.last_status = status
        suffix = f' | {reason}' if reason else ''
        print(
            f'cmd_vel: linear={self.linear:+.3f} m/s  '
            f'angular={self.angular:+.3f} rad/s{suffix}\n'
            f'setpoints: drive={self.linear_speed:.2f} m/s  '
            f'curve={self.angular_speed:.2f} rad/s  '
            f'pivot={self.pivot_speed:.2f} rad/s',
            flush=True)


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('safe_keyboard_teleop requires an interactive TTY')
    rclpy.init(args=args)
    node = SafeKeyboardTeleop()
    previous_terminal = termios.tcgetattr(sys.stdin)
    period = 1.0 / max(node.publish_rate, 1.0)
    print(HELP)
    if node.mapping_mode:
        print(
            'MAPPING MODE: drive 0.08-0.12 m/s, curve <=0.18 rad/s; '
            'pivot is independently adjustable.')
    node.stop('READY; motors commanded to zero')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], period)
            if readable:
                key = sys.stdin.read(1)
                if key in ('\x1b', '\x03'):
                    break
                if node.set_key(key):
                    node.show_status()
            node.expire_command()
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop('EXIT')
            for _ in range(4):
                node.publish()
                time.sleep(0.05)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, previous_terminal)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
