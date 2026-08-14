"""Acceleration- and braking-limited velocity controller."""

from __future__ import annotations

import time
from typing import Optional

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions


def step_velocity(
    current: float,
    target: float,
    dt: float,
    max_acceleration: float,
    max_deceleration: float,
) -> float:
    """Move one velocity component toward target without overshooting it."""
    if dt <= 0.0:
        return current
    delta = target - current
    if abs(delta) < 1e-12:
        return target

    changing_sign = current != 0.0 and target != 0.0 and current * target < 0.0
    same_direction = current == 0.0 or target == 0.0 or current * target > 0.0
    increasing_speed = same_direction and abs(target) > abs(current)
    rate_limit = max_acceleration if increasing_speed else max_deceleration
    maximum_change = rate_limit * dt
    if changing_sign and abs(current) <= maximum_change:
        return 0.0
    if abs(delta) <= maximum_change:
        return target
    return current + (maximum_change if delta > 0.0 else -maximum_change)


def bounded_target(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, value))


class VelocityLimiter(Node):
    """Convert target Twist commands into safe, rate-limited Twist commands."""

    def __init__(self) -> None:
        super().__init__('velocity_limiter')
        self.declare_parameter('input_topic', '/cmd_vel/target')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('input_timeout', 0.50)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('max_acceleration', 1.2)
        self.declare_parameter('max_deceleration', 6.0)
        self.declare_parameter('max_angular_velocity', 3.0)
        self.declare_parameter('max_angular_acceleration', 3.0)
        self.declare_parameter('max_angular_deceleration', 6.0)

        self.update_rate = float(self.get_parameter('update_rate').value)
        self.input_timeout = float(self.get_parameter('input_timeout').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.max_acceleration = float(self.get_parameter('max_acceleration').value)
        self.max_deceleration = float(self.get_parameter('max_deceleration').value)
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value
        )
        self.max_angular_acceleration = float(
            self.get_parameter('max_angular_acceleration').value
        )
        self.max_angular_deceleration = float(
            self.get_parameter('max_angular_deceleration').value
        )
        if min(
            self.update_rate,
            self.input_timeout,
            self.max_velocity,
            self.max_acceleration,
            self.max_deceleration,
            self.max_angular_velocity,
            self.max_angular_acceleration,
            self.max_angular_deceleration,
        ) <= 0.0:
            raise ValueError('Velocity limiter parameters must be positive.')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.create_subscription(Twist, input_topic, self._on_target, 10)
        self.output_publisher = self.create_publisher(Twist, output_topic, 10)

        self._target_linear = 0.0
        self._target_angular = 0.0
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._last_input_at = float('-inf')
        self._last_update_at = time.monotonic()
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self.create_timer(1.0 / self.update_rate, self._update)

    def _on_parameters_changed(
        self,
        parameters: list[Parameter],
    ) -> SetParametersResult:
        runtime_parameters = {
            'input_timeout': 'input_timeout',
            'max_velocity': 'max_velocity',
            'max_acceleration': 'max_acceleration',
            'max_deceleration': 'max_deceleration',
            'max_angular_velocity': 'max_angular_velocity',
            'max_angular_acceleration': 'max_angular_acceleration',
            'max_angular_deceleration': 'max_angular_deceleration',
        }
        restart_parameters = {'input_topic', 'output_topic', 'update_rate'}
        updates: dict[str, float] = {}

        for parameter in parameters:
            if parameter.name in restart_parameters:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} requires restarting the node.',
                )
            attribute = runtime_parameters.get(parameter.name)
            if attribute is None:
                continue
            if isinstance(parameter.value, bool) or not isinstance(
                parameter.value,
                (int, float),
            ):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must be numeric.',
                )
            if parameter.value <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must be positive.',
                )
            updates[attribute] = float(parameter.value)

        for attribute, value in updates.items():
            setattr(self, attribute, value)
        self._target_linear = bounded_target(
            self._target_linear,
            self.max_velocity,
        )
        self._target_angular = bounded_target(
            self._target_angular,
            self.max_angular_velocity,
        )
        return SetParametersResult(successful=True)

    def _on_target(self, message: Twist) -> None:
        self._target_linear = bounded_target(message.linear.x, self.max_velocity)
        self._target_angular = bounded_target(
            message.angular.z,
            self.max_angular_velocity,
        )
        self._last_input_at = time.monotonic()

    def _update(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last_update_at, 0.1)
        self._last_update_at = now
        target_linear = self._target_linear
        target_angular = self._target_angular
        if now - self._last_input_at > self.input_timeout:
            target_linear = 0.0
            target_angular = 0.0

        self._current_linear = step_velocity(
            self._current_linear,
            target_linear,
            dt,
            self.max_acceleration,
            self.max_deceleration,
        )
        self._current_angular = step_velocity(
            self._current_angular,
            target_angular,
            dt,
            self.max_angular_acceleration,
            self.max_angular_deceleration,
        )
        message = Twist()
        message.linear.x = self._current_linear
        message.angular.z = self._current_angular
        self.output_publisher.publish(message)

    def publish_stop(self) -> None:
        self.output_publisher.publish(Twist())


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityLimiter()
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
