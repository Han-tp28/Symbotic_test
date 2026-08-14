import pytest

from symbotic_control.keyboard_teleop import ARROW_DOWN
from symbotic_control.keyboard_teleop import ARROW_LEFT
from symbotic_control.keyboard_teleop import ARROW_RIGHT
from symbotic_control.keyboard_teleop import ARROW_UP
from symbotic_control.keyboard_teleop import command_for_key


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
