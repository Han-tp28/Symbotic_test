"""Terminal keyboard teleoperation for the differential-drive robot."""

from __future__ import annotations

import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool


ARROW_UP = '\x1b[A'
ARROW_DOWN = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT = '\x1b[D'


def command_for_key(
    key: str,
    linear_speed: float,
    angular_speed: float,
) -> Optional[tuple[float, float]]:
    """Return ``(linear_x, angular_z)`` for a movement key."""
    normalized = key.lower() if len(key) == 1 else key
    commands = {
        'w': (linear_speed, 0.0),
        ARROW_UP: (linear_speed, 0.0),
        's': (-linear_speed, 0.0),
        ARROW_DOWN: (-linear_speed, 0.0),
        'a': (0.0, angular_speed),
        ARROW_LEFT: (0.0, angular_speed),
        'd': (0.0, -angular_speed),
        ARROW_RIGHT: (0.0, -angular_speed),
    }
    return commands.get(normalized)


def updated_command_for_key(
    key: str,
    current_linear_x: float,
    current_angular_z: float,
    linear_speed: float,
    angular_speed: float,
) -> Optional[tuple[float, float]]:
    """Update only the velocity axis controlled by ``key``.

    Keeping the other axis allows forward/backward and steering keys to form
    one combined Twist command, for example W+A -> forward-left arc.
    """
    key_command = command_for_key(key, linear_speed, angular_speed)
    if key_command is None:
        return None
    if key_command[0] != 0.0:
        return key_command[0], current_angular_z
    return current_linear_x, key_command[1]


def read_key(timeout: float) -> Optional[str]:
    """Read one terminal key, including a complete arrow-key sequence."""
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return None

    key = sys.stdin.read(1)
    if key != '\x1b':
        return key

    sequence = key
    for _ in range(2):
        readable, _, _ = select.select([sys.stdin], [], [], 0.02)
        if not readable:
            break
        sequence += sys.stdin.read(1)
    return sequence


class RawTerminal:
    """Temporarily place stdin in cbreak mode and always restore it."""

    def __init__(self) -> None:
        self._file_descriptor = sys.stdin.fileno()
        self._settings = None

    def __enter__(self) -> 'RawTerminal':
        self._settings = termios.tcgetattr(self._file_descriptor)
        tty.setcbreak(self._file_descriptor)
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        if self._settings is not None:
            termios.tcsetattr(
                self._file_descriptor,
                termios.TCSADRAIN,
                self._settings,
            )


class KeyboardTeleop(Node):
    """Publish manual commands and a latched emergency-stop state."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('key_timeout', 0.70)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('manual_topic', '/cmd_vel/manual')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.key_timeout = float(self.get_parameter('key_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        if min(
            self.linear_speed,
            self.angular_speed,
            self.key_timeout,
            self.publish_rate,
        ) <= 0.0:
            raise ValueError('Teleop speed, timeout and publish rate must be positive.')

        manual_topic = str(self.get_parameter('manual_topic').value)
        emergency_stop_topic = str(
            self.get_parameter('emergency_stop_topic').value
        )

        emergency_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.command_publisher = self.create_publisher(Twist, manual_topic, 10)
        self.emergency_stop_publisher = self.create_publisher(
            Bool,
            emergency_stop_topic,
            emergency_qos,
        )

        self._linear_x = 0.0
        self._angular_z = 0.0
        self._linear_active_until = 0.0
        self._angular_active_until = 0.0
        self._emergency_stop = False

        self.publish_emergency_stop(False)
        self.publish_command(0.0, 0.0)

    def publish_command(self, linear_x: float, angular_z: float) -> None:
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.command_publisher.publish(message)

    def publish_emergency_stop(self, enabled: bool) -> None:
        self._emergency_stop = enabled
        message = Bool()
        message.data = enabled
        self.emergency_stop_publisher.publish(message)

    def handle_key(self, key: str) -> bool:
        """Process a key and return ``False`` when the node should exit."""
        if key in ('q', 'Q'):
            self.stop(latch_emergency=True)
            return False

        if key == ' ':
            self.stop(latch_emergency=True)
            self.get_logger().warn('Emergency stop enabled. Press a movement key to clear it.')
            return True

        key_command = command_for_key(key, self.linear_speed, self.angular_speed)
        if key_command is None:
            return True

        if self._emergency_stop:
            self.publish_emergency_stop(False)
            self.get_logger().info('Emergency stop cleared by manual input.')

        updated_command = updated_command_for_key(
            key,
            self._linear_x,
            self._angular_z,
            self.linear_speed,
            self.angular_speed,
        )
        self._linear_x, self._angular_z = updated_command

        active_until = time.monotonic() + self.key_timeout
        if key_command[0] != 0.0:
            self._linear_active_until = active_until
        else:
            self._angular_active_until = active_until
        self.publish_command(self._linear_x, self._angular_z)
        return True

    def tick(self) -> None:
        """Maintain a held command and stop after keyboard repeat times out."""
        if self._emergency_stop:
            self.publish_command(0.0, 0.0)
            return

        now = time.monotonic()
        command_changed = False
        if self._linear_x != 0.0 and now > self._linear_active_until:
            self._linear_x = 0.0
            command_changed = True
        if self._angular_z != 0.0 and now > self._angular_active_until:
            self._angular_z = 0.0
            command_changed = True

        if self._linear_x != 0.0 or self._angular_z != 0.0 or command_changed:
            self.publish_command(self._linear_x, self._angular_z)

    def stop(self, latch_emergency: bool) -> None:
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._linear_active_until = 0.0
        self._angular_active_until = 0.0
        self.publish_command(0.0, 0.0)
        if latch_emergency:
            self.publish_emergency_stop(True)


HELP_TEXT = """
Keyboard teleoperation
----------------------
Hold W / Up Arrow       : move forward
Hold S / Down Arrow     : move backward
Hold A / Left Arrow     : steer left
Hold D / Right Arrow    : steer right
Combine move + steer    : drive along a curved path
Space                   : latched emergency stop
Q                       : stop and quit

Manual input has priority over autonomous commands.
"""


def main(args=None) -> None:
    # Keep the context alive during Ctrl+C cleanup so the final stop command
    # and latched emergency-stop state can still be published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()

    if not sys.stdin.isatty():
        node.get_logger().error('keyboard_teleop must run in an interactive terminal.')
        node.stop(latch_emergency=True)
        node.destroy_node()
        rclpy.shutdown()
        return

    keep_running = True
    try:
        print(HELP_TEXT, flush=True)
        with RawTerminal():
            while rclpy.ok() and keep_running:
                rclpy.spin_once(node, timeout_sec=0.0)
                key = read_key(1.0 / node.publish_rate)
                if key is not None:
                    keep_running = node.handle_key(key)
                node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop(latch_emergency=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
