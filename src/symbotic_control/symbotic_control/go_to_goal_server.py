"""Go-to-goal action server for the differential-drive robot."""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from symbotic_interfaces.action import GoToGoal


Pose2D = tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def compute_control(
    pose: Pose2D,
    target_x: float,
    target_y: float,
    tolerance: float,
    linear_gain: float,
    angular_gain: float,
    max_linear_speed: float,
    max_angular_speed: float,
    rotate_in_place_angle: float,
) -> tuple[float, float, float, float]:
    """Return linear speed, angular speed, distance and heading error."""
    x, y, yaw = pose
    delta_x = target_x - x
    delta_y = target_y - y
    distance = math.hypot(delta_x, delta_y)
    heading = math.atan2(delta_y, delta_x)
    heading_error = normalize_angle(heading - yaw)

    if distance <= tolerance:
        return 0.0, 0.0, distance, heading_error

    angular = max(
        -max_angular_speed,
        min(max_angular_speed, angular_gain * heading_error),
    )
    if abs(heading_error) >= rotate_in_place_angle:
        linear = 0.0
    else:
        alignment = max(0.0, math.cos(heading_error))
        linear = min(max_linear_speed, linear_gain * distance) * alignment
    return linear, angular, distance, heading_error


class GoToGoalServer(Node):
    """Drive toward one odom-frame coordinate and report action progress."""

    def __init__(self) -> None:
        super().__init__('go_to_goal_server')
        self.declare_parameter('action_name', '/go_to_goal')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('command_topic', '/cmd_vel/autonomous')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('odom_timeout', 1.0)
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 2.5)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('rotate_in_place_angle', 0.60)

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.rotate_in_place_angle = float(
            self.get_parameter('rotate_in_place_angle').value
        )
        if min(
            self.control_rate,
            self.odom_timeout,
            self.linear_gain,
            self.angular_gain,
            self.max_linear_speed,
            self.max_angular_speed,
            self.rotate_in_place_angle,
        ) <= 0.0:
            raise ValueError('Go-to-goal controller parameters must be positive.')

        callback_group = ReentrantCallbackGroup()
        odom_topic = str(self.get_parameter('odom_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)
        action_name = str(self.get_parameter('action_name').value)
        self.command_publisher = self.create_publisher(Twist, command_topic, 10)
        self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odometry,
            10,
            callback_group=callback_group,
        )

        self._state_lock = threading.Lock()
        self._pose: Optional[Pose2D] = None
        self._odom_received_at = float('-inf')
        self._goal_active = False
        self._action_server = ActionServer(
            self,
            GoToGoal,
            action_name,
            execute_callback=self._execute_goal,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=callback_group,
        )

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        pose = (
            position.x,
            position.y,
            yaw_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        with self._state_lock:
            self._pose = pose
            self._odom_received_at = time.monotonic()

    def _on_goal(self, request: GoToGoal.Goal) -> GoalResponse:
        values = (request.x, request.y, request.tolerance, request.timeout_sec)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn('Rejected goal with non-finite values.')
            return GoalResponse.REJECT
        if request.tolerance <= 0.0 or request.timeout_sec <= 0.0:
            self.get_logger().warn('Goal tolerance and timeout must be positive.')
            return GoalResponse.REJECT

        with self._state_lock:
            if self._goal_active:
                self.get_logger().warn('Rejected goal because another goal is active.')
                return GoalResponse.REJECT
            self._goal_active = True
        self.get_logger().info(
            f'Accepted goal ({request.x:.3f}, {request.y:.3f}), '
            f'tolerance={request.tolerance:.3f} m, '
            f'timeout={request.timeout_sec:.1f} s.'
        )
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request accepted.')
        return CancelResponse.ACCEPT

    def _pose_snapshot(self) -> tuple[Optional[Pose2D], float]:
        with self._state_lock:
            return self._pose, self._odom_received_at

    def _publish_command(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.command_publisher.publish(message)

    def publish_stop(self) -> None:
        self._publish_command(0.0, 0.0)

    @staticmethod
    def _result(
        success: bool,
        message: str,
        pose: Optional[Pose2D],
        target_x: float,
        target_y: float,
    ) -> GoToGoal.Result:
        result = GoToGoal.Result()
        result.success = success
        result.message = message
        if pose is not None:
            result.final_x = pose[0]
            result.final_y = pose[1]
            result.final_distance = math.hypot(target_x - pose[0], target_y - pose[1])
        else:
            result.final_x = math.nan
            result.final_y = math.nan
            result.final_distance = math.inf
        return result

    def _execute_goal(self, goal_handle) -> GoToGoal.Result:
        request = goal_handle.request
        started_at = time.monotonic()
        period = 1.0 / self.control_rate
        latest_pose: Optional[Pose2D] = None
        try:
            while rclpy.ok():
                loop_started_at = time.monotonic()
                elapsed = loop_started_at - started_at
                latest_pose, odom_received_at = self._pose_snapshot()

                if goal_handle.is_cancel_requested:
                    self.publish_stop()
                    goal_handle.canceled()
                    return self._result(
                        False,
                        'Goal canceled by user.',
                        latest_pose,
                        request.x,
                        request.y,
                    )

                if elapsed >= request.timeout_sec:
                    self.publish_stop()
                    goal_handle.abort()
                    return self._result(
                        False,
                        f'Goal timed out after {elapsed:.2f} s.',
                        latest_pose,
                        request.x,
                        request.y,
                    )

                if latest_pose is None:
                    if elapsed >= self.odom_timeout:
                        self.publish_stop()
                        goal_handle.abort()
                        return self._result(
                            False,
                            'No odometry received.',
                            None,
                            request.x,
                            request.y,
                        )
                elif loop_started_at - odom_received_at > self.odom_timeout:
                    self.publish_stop()
                    goal_handle.abort()
                    return self._result(
                        False,
                        'Odometry became stale.',
                        latest_pose,
                        request.x,
                        request.y,
                    )
                else:
                    linear, angular, distance, heading_error = compute_control(
                        latest_pose,
                        request.x,
                        request.y,
                        request.tolerance,
                        self.linear_gain,
                        self.angular_gain,
                        self.max_linear_speed,
                        self.max_angular_speed,
                        self.rotate_in_place_angle,
                    )
                    if distance <= request.tolerance:
                        self.publish_stop()
                        goal_handle.succeed()
                        return self._result(
                            True,
                            'Goal reached within tolerance.',
                            latest_pose,
                            request.x,
                            request.y,
                        )

                    self._publish_command(linear, angular)
                    feedback = GoToGoal.Feedback()
                    feedback.current_x = latest_pose[0]
                    feedback.current_y = latest_pose[1]
                    feedback.distance_remaining = distance
                    feedback.heading_error = heading_error
                    feedback.elapsed_time = elapsed
                    goal_handle.publish_feedback(feedback)

                remaining = period - (time.monotonic() - loop_started_at)
                if remaining > 0.0:
                    time.sleep(remaining)

            goal_handle.abort()
            return self._result(
                False,
                'ROS shutdown before goal completion.',
                latest_pose,
                request.x,
                request.y,
            )
        finally:
            self.publish_stop()
            with self._state_lock:
                self._goal_active = False

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoToGoalServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
