from symbotic_control.command_arbiter import select_velocity


def test_emergency_stop_has_highest_priority():
    command, source = select_velocity(
        True,
        (2.0, 0.0),
        0.0,
        0.8,
        (1.0, 0.5),
        0.0,
        0.5,
    )
    assert command == (0.0, 0.0)
    assert source == 'emergency_stop'


def test_fresh_manual_command_beats_autonomous_command():
    command, source = select_velocity(
        False,
        (0.0, 1.5),
        0.1,
        0.8,
        (1.0, 0.0),
        0.1,
        0.5,
    )
    assert command == (0.0, 1.5)
    assert source == 'manual'


def test_autonomous_command_runs_after_manual_timeout():
    command, source = select_velocity(
        False,
        (0.0, 1.5),
        1.0,
        0.8,
        (1.0, 0.0),
        0.1,
        0.5,
    )
    assert command == (1.0, 0.0)
    assert source == 'autonomous'


def test_stale_sources_produce_zero_velocity():
    command, source = select_velocity(
        False,
        (2.0, 0.0),
        1.0,
        0.8,
        (1.0, 0.0),
        1.0,
        0.5,
    )
    assert command == (0.0, 0.0)
    assert source == 'idle'
