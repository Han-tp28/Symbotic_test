import pytest

from symbotic_control.keyboard_teleop import ARROW_DOWN
from symbotic_control.keyboard_teleop import ARROW_LEFT
from symbotic_control.keyboard_teleop import ARROW_RIGHT
from symbotic_control.keyboard_teleop import ARROW_UP
from symbotic_control.keyboard_teleop import command_for_key
from symbotic_control.keyboard_teleop import updated_command_for_key


@pytest.mark.parametrize(
    ('key', 'expected'),
    [
        ('w', (2.0, 0.0)),
        ('W', (2.0, 0.0)),
        (ARROW_UP, (2.0, 0.0)),
        ('s', (-2.0, 0.0)),
        (ARROW_DOWN, (-2.0, 0.0)),
        ('a', (0.0, 1.5)),
        (ARROW_LEFT, (0.0, 1.5)),
        ('d', (0.0, -1.5)),
        (ARROW_RIGHT, (0.0, -1.5)),
    ],
)
def test_movement_key_mapping(key, expected):
    assert command_for_key(key, 2.0, 1.5) == expected


def test_unknown_key_has_no_command():
    assert command_for_key('x', 2.0, 1.5) is None


@pytest.mark.parametrize(
    ('linear_key', 'steering_key', 'expected'),
    [
        ('w', 'a', (2.0, 1.5)),
        ('w', 'd', (2.0, -1.5)),
        ('s', 'a', (-2.0, 1.5)),
        ('s', 'd', (-2.0, -1.5)),
        (ARROW_UP, ARROW_LEFT, (2.0, 1.5)),
        (ARROW_UP, ARROW_RIGHT, (2.0, -1.5)),
    ],
)
def test_movement_and_steering_are_combined(linear_key, steering_key, expected):
    command = updated_command_for_key(linear_key, 0.0, 0.0, 2.0, 1.5)
    command = updated_command_for_key(steering_key, *command, 2.0, 1.5)
    assert command == expected


def test_linear_key_preserves_active_steering():
    assert updated_command_for_key('w', 0.0, 1.5, 2.0, 1.5) == (2.0, 1.5)
