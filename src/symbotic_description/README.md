# symbotic_description

Robot model, Gazebo world and launch assets will be added in the next
implementation milestone.

## URDF test file

The expanded URDF snapshot is available at:

```text
urdf/naked/symbotic_diff_drive.urdf
```

Validate it with:

```bash
check_urdf src/symbotic_description/urdf/naked/symbotic_diff_drive.urdf
```

The source of truth remains `config/robot.yaml` and
`urdf/differential_drive.urdf.xacro`. Regenerate the plain URDF after changing
those files with:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro src/symbotic_description/urdf/differential_drive.urdf.xacro \
  > src/symbotic_description/urdf/naked/symbotic_diff_drive.urdf
```

## Gazebo simulation

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch symbotic_description simulation.launch.py
```

Run without the Gazebo GUI:

```bash
ros2 launch symbotic_description simulation.launch.py headless:=true
```

The launch file exposes `/cmd_vel`, `/odom`, `/tf`, `/joint_states` and
`/clock` through `ros_gz_bridge`.
