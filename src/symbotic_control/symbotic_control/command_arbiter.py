"""Priority arbitration between manual and autonomous velocity commands."""

from __future__ import annotations

import time
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool


Velocity = tuple[float, float]


def select_velocity(
    emergency_stop: bool,
    manual_command: Optional[Velocity],
    manual_age: float,
    manual_timeout: float,
    autonomous_command: Optional[Velocity],
    autonomous_age: float,
    autonomous_timeout: float,
) -> tuple[Velocity, str]:
    """Select a safe velocity and report which source owns the output."""
    if emergency_stop:
        return (0.0, 0.0), 'emergency_stop'
    if manual_command is not None and manual_age <= manual_timeout:
        return manual_command, 'manual'
    if autonomous_command is not None and autonomous_age <= autonomous_timeout:
        return autonomous_command, 'autonomous'
    return (0.0, 0.0), 'idle'


class CommandArbiter(Node):
    """Publish manual commands first, then autonomous commands, then zero."""

    def __init__(self) -> None:
        super().__init__('command_arbiter')

        self.declare_parameter('output_rate', 20.0)
        self.declare_parameter('manual_timeout', 0.80)
        self.declare_parameter('autonomous_timeout', 0.50)
        self.declare_parameter('manual_topic', '/cmd_vel/manual')
        self.declare_parameter('autonomous_topic', '/cmd_vel/autonomous')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')

        self.output_rate = float(self.get_parameter('output_rate').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)
        self.autonomous_timeout = float(
            self.get_parameter('autonomous_timeout').value
        )
        if min(
            self.output_rate,
            self.manual_timeout,
            self.autonomous_timeout,
        ) <= 0.0:
            raise ValueError('Arbiter rates and timeouts must be positive.')

        manual_topic = str(self.get_parameter('manual_topic').value)
        autonomous_topic = str(self.get_parameter('autonomous_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        emergency_stop_topic = str(
            self.get_parameter('emergency_stop_topic').value
        )

        emergency_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Twist, manual_topic, self._on_manual, 10)
        self.create_subscription(Twist, autonomous_topic, self._on_autonomous, 10)
        self.create_subscription(
            Bool,
            emergency_stop_topic,
            self._on_emergency_stop,
            emergency_qos,
        )
        self.output_publisher = self.create_publisher(Twist, output_topic, 10)

        self._manual_command: Optional[Velocity] = None
        self._manual_received_at = float('-inf')
        self._autonomous_command: Optional[Velocity] = None
        self._autonomous_received_at = float('-inf')
        self._emergency_stop = False
        self._active_source = ''

        self.create_timer(1.0 / self.output_rate, self._publish_selected_command)

    def _on_manual(self, message: Twist) -> None:
        self._manual_command = (message.linear.x, message.angular.z)
        self._manual_received_at = time.monotonic()

    def _on_autonomous(self, message: Twist) -> None:
        self._autonomous_command = (message.linear.x, message.angular.z)
        self._autonomous_received_at = time.monotonic()

    def _on_emergency_stop(self, message: Bool) -> None:
        self._emergency_stop = message.data

    def _publish_selected_command(self) -> None:
        now = time.monotonic()
        command, source = select_velocity(
            emergency_stop=self._emergency_stop,
            manual_command=self._manual_command,
            manual_age=now - self._manual_received_at,
            manual_timeout=self.manual_timeout,
            autonomous_command=self._autonomous_command,
            autonomous_age=now - self._autonomous_received_at,
            autonomous_timeout=self.autonomous_timeout,
        )

        message = Twist()
        message.linear.x = command[0]
        message.angular.z = command[1]
        self.output_publisher.publish(message)

        if source != self._active_source:
            self._active_source = source
            self.get_logger().info(f'Command source: {source}')

    def publish_stop(self) -> None:
        self.output_publisher.publish(Twist())


def main(args=None) -> None:
    # Keep the context alive during Ctrl+C cleanup so a final zero velocity
    # can be published before shutting ROS down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CommandArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
