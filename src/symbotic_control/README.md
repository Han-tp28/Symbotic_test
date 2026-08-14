# symbotic_control

## Keyboard teleoperation

Start the simulator in terminal 1:

```bash
ros2 launch symbotic_description simulation.launch.py
```

Start command arbitration in terminal 2:

```bash
PARAMS="$(ros2 pkg prefix symbotic_control)/share/symbotic_control/config/teleop.yaml"
ros2 run symbotic_control command_arbiter --ros-args --params-file "$PARAMS"
```

Start keyboard input in terminal 3:

```bash
PARAMS="$(ros2 pkg prefix symbotic_control)/share/symbotic_control/config/teleop.yaml"
ros2 run symbotic_control keyboard_teleop --ros-args --params-file "$PARAMS"
```

Controls:

- Hold `W` or Up Arrow: move forward.
- Hold `S` or Down Arrow: move backward.
- Hold `A` or Left Arrow: rotate left.
- Hold `D` or Right Arrow: rotate right.
- Press Space: latch emergency stop.
- Press a movement key: clear emergency stop and take manual control.
- Press `Q`: stop and exit.

The arbiter publishes zero when all command sources time out. Fresh manual input
always overrides `/cmd_vel/autonomous`.
