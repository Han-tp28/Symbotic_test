import math

import pytest

from symbotic_control.go_to_goal_server import compute_control
from symbotic_control.go_to_goal_server import normalize_angle
from symbotic_control.go_to_goal_server import yaw_from_quaternion


CONTROLLER = {
    'linear_gain': 1.0,
    'angular_gain': 2.5,
    'max_linear_speed': 2.0,
    'max_angular_speed': 1.5,
    'rotate_in_place_angle': 0.60,
}


def control(pose, target_x, target_y, tolerance=0.10):
    return compute_control(
        pose,
        target_x,
        target_y,
        tolerance,
        **CONTROLLER,
    )


def test_drives_straight_toward_goal():
    linear, angular, distance, heading_error = control((0.0, 0.0, 0.0), 2.0, 0.0)
    assert linear == pytest.approx(2.0)
    assert angular == pytest.approx(0.0)
    assert distance == pytest.approx(2.0)
    assert heading_error == pytest.approx(0.0)


def test_rotates_in_place_when_goal_is_behind():
    linear, angular, _, _ = control((0.0, 0.0, 0.0), -1.0, 0.0)
    assert linear == 0.0
    assert abs(angular) == pytest.approx(1.5)


def test_combines_forward_and_turning_for_small_heading_error():
    linear, angular, _, _ = control((0.0, 0.0, 0.0), 2.0, 0.5)
    assert linear > 0.0
    assert angular > 0.0


def test_stops_inside_goal_tolerance():
    linear, angular, distance, _ = control((0.95, 0.0, 0.0), 1.0, 0.0)
    assert (linear, angular) == (0.0, 0.0)
    assert distance < 0.10


@pytest.mark.parametrize(
    ('angle', 'expected'),
    [(3.0 * math.pi, math.pi), (-3.0 * math.pi, -math.pi)],
)
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected)


def test_yaw_from_quaternion():
    half_angle = math.pi / 4.0
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
    assert yaw == pytest.approx(math.pi / 2.0)
