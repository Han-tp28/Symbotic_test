import math

import pytest

from symbotic_control.velocity_limiter import bounded_target
from symbotic_control.velocity_limiter import step_velocity


def simulate(current, target, dt, duration, acceleration=1.2, deceleration=6.0):
    values = []
    for _ in range(math.ceil(duration / dt)):
        current = step_velocity(
            current,
            target,
            dt,
            acceleration,
            deceleration,
        )
        values.append(current)
    return values


def test_acceleration_reaches_five_meters_per_second_within_five_seconds():
    values = simulate(0.0, 5.0, 0.01, 5.0)
    first_reach = next(index for index, value in enumerate(values) if value >= 5.0)
    assert (first_reach + 1) * 0.01 <= 5.0
    assert values[-1] == pytest.approx(5.0)


def test_braking_stops_five_meters_per_second_within_one_second():
    values = simulate(5.0, 0.0, 0.01, 1.0)
    first_stop = next(index for index, value in enumerate(values) if value == 0.0)
    assert (first_stop + 1) * 0.01 <= 1.0


def test_sign_change_brakes_before_reversing():
    braking_values = simulate(2.0, -2.0, 0.01, 0.30)
    reversing_values = simulate(2.0, -2.0, 0.01, 1.0)
    assert min(braking_values) >= 0.0
    assert reversing_values[-1] < 0.0


@pytest.mark.parametrize('value', [-9.0, 9.0])
def test_target_velocity_is_bounded(value):
    assert abs(bounded_target(value, 5.0)) == 5.0
