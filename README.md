# Configurable Differential-Drive Robot Simulation

A ROS 2 Jazzy and Gazebo Harmonic simulation of a configurable differential-drive
mobile robot. The project includes keyboard teleoperation, command-source
arbitration, explicit acceleration and braking limits, runtime controller
configuration, and a cancelable Go-To-Goal action with feedback and timeout.

## Requirements covered

| Requirement | Implementation |
| --- | --- |
| Differential-drive robot | Xacro/URDF model with two drive wheels and symmetric ball casters |
| Accurate kinematics | Gazebo DiffDrive plugin configured from the same wheel radius and track width as the model |
| Keyboard control | WASD or arrow keys, Space emergency stop, Q to quit |
| Reach 5 m/s within 5 s | 1.2 m/s² acceleration reaches 5 m/s in approximately 4.17 s |
| Stop within 1 s | 6.0 m/s² deceleration stops from 5 m/s in approximately 0.83 s |
| Configurable physical parameters | `robot_params.yaml` for geometry, dynamics, and initial controller limits |
| Interactive controller configuration | ROS 2 parameters can change velocity, acceleration, and deceleration at runtime |
| Go-To-Goal | Custom action server with tolerance, feedback, cancel, timeout, and safe stop |
| Manual priority | Command arbiter prioritizes emergency stop, then manual, then autonomous input |

## Target environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / Gz-Sim 8
- `ros_gz`
- `colcon`

The project was developed and tested in this environment. A Docker installation
based on `ros:jazzy-ros-base` may also be used, but Docker files are not required
to run or evaluate the source code.

## Workspace layout

```text
Symbotic_test/
├── src/
│   ├── symbotic_description/   # Xacro, URDF, YAML, Gazebo world and launch
│   ├── symbotic_interfaces/    # GoToGoal.action
│   └── symbotic_control/       # Teleop, arbiter, limiter and action server
├── README.md
└── .gitignore
```

## Install dependencies

Install ROS 2 Jazzy using the official ROS instructions, then install the
workspace dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher
```

Initialize `rosdep` once if it has not been initialized on the machine:

```bash
sudo rosdep init
rosdep update
```

## Build

```bash
git clone https://github.com/Han-tp28/Symbotic_test.git
cd Symbotic_test

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Every new terminal must source both ROS 2 and this workspace:

```bash
cd /path/to/Symbotic_test
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

If Conda is active and reports an `_rclpy_pybind11` Python-version error, run
`conda deactivate` before sourcing ROS 2.

## Control architecture

```text
keyboard_teleop  -- /cmd_vel/manual -----+
                                           |
go_to_goal      -- /cmd_vel/autonomous --+--> command_arbiter
                                                   |
                                             /cmd_vel/target
                                                   |
                                           velocity_limiter
                                                   |
                                                /cmd_vel
                                                   |
                                            ros_gz_bridge
                                                   |
                                          Gazebo DiffDrive

/emergency_stop -------- highest priority ---------^
```

The simulation launch starts Gazebo, robot state publishing, the ROS-Gazebo
bridge, and `velocity_limiter`. Start `command_arbiter` separately for both
manual and autonomous control.

## Run keyboard teleoperation

### Terminal 1: simulation

```bash
cd /path/to/Symbotic_test
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch symbotic_description simulation.launch.py
```

Use `headless:=true` to run without the Gazebo GUI:

```bash
ros2 launch symbotic_description simulation.launch.py headless:=true
```

### Terminal 2: command arbiter

```bash
cd /path/to/Symbotic_test
source /opt/ros/jazzy/setup.bash
source install/setup.bash

PARAMS="$(ros2 pkg prefix symbotic_control)/share/symbotic_control/config/teleop.yaml"
ros2 run symbotic_control command_arbiter --ros-args --params-file "$PARAMS"
```

### Terminal 3: keyboard input

```bash
cd /path/to/Symbotic_test
source /opt/ros/jazzy/setup.bash
source install/setup.bash

PARAMS="$(ros2 pkg prefix symbotic_control)/share/symbotic_control/config/teleop.yaml"
ros2 run symbotic_control keyboard_teleop --ros-args --params-file "$PARAMS"
```

Controls:

| Key | Command |
| --- | --- |
| `W` / Up | Forward |
| `S` / Down | Reverse |
| `A` / Left | Turn left |
| `D` / Right | Turn right |
| Movement + turn | Follow a curved path |
| Space | Latch emergency stop |
| Any movement key | Clear emergency stop and take manual control |
| `Q` | Stop and exit |

Keyboard input is time-limited: if key updates stop, the manual command expires
and the robot stops. Fresh manual input always overrides an active autonomous
goal.

## Run Go-To-Goal

Keep the simulation and command arbiter from Terminals 1 and 2 running.

### Terminal 3: action server

```bash
cd /path/to/Symbotic_test
source /opt/ros/jazzy/setup.bash
source install/setup.bash

PARAMS="$(ros2 pkg prefix symbotic_control)/share/symbotic_control/config/teleop.yaml"
ros2 run symbotic_control go_to_goal_server --ros-args --params-file "$PARAMS"
```

### Terminal 4: send a goal

```bash
cd /path/to/Symbotic_test
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 action send_goal /go_to_goal \
  symbotic_interfaces/action/GoToGoal \
  "{x: 2.0, y: 1.0, tolerance: 0.15, timeout_sec: 30.0}" \
  --feedback
```

Coordinates are expressed in the `odom` frame. The controller rotates toward
the destination, drives forward while correcting heading, and succeeds only
when the remaining Euclidean distance is within the requested tolerance.

The action definition is:

```text
Goal:     x, y, tolerance, timeout_sec
Feedback: current_x, current_y, distance_remaining, heading_error, elapsed_time
Result:   success, message, final_x, final_y, final_distance
```

The server accepts cancellation from ROS 2 action clients. Cancellation,
timeout, stale odometry, and shutdown all publish a zero autonomous command.
Only one goal is accepted at a time.

Useful action inspection commands:

```bash
ros2 action list -t
ros2 action info /go_to_goal
ros2 interface show symbotic_interfaces/action/GoToGoal
```

## Configure the robot

The source of truth is
[`src/symbotic_description/config/robot_params.yaml`](src/symbotic_description/config/robot_params.yaml).

```yaml
robot:
  geometry:
    wheel_radius: 0.15
    track_width: 0.50
    wheelbase: 0.30
    chassis_length: 0.80
    chassis_width: 0.60
    chassis_height: 0.25

  controller:
    max_velocity: 5.0
    max_acceleration: 1.2
    max_deceleration: 6.0
```

Geometry and mass changes require rebuilding and relaunching so Xacro can
regenerate and Gazebo can respawn the rigid bodies:

```bash
colcon build --symlink-install --packages-select symbotic_description
source install/setup.bash
ros2 launch symbotic_description simulation.launch.py
```

Controller limits can be inspected or changed while the simulation is running:

```bash
ros2 param get /velocity_limiter max_velocity
ros2 param set /velocity_limiter max_velocity 3.0
ros2 param set /velocity_limiter max_acceleration 1.5
ros2 param set /velocity_limiter max_deceleration 7.0
```

Only positive values are accepted. Runtime changes are not written back to the
YAML file.

## Kinematics

For commanded robot linear velocity `v`, yaw rate `ω`, track width `L`, and
wheel radius `r`:

```text
v_right = v + ωL/2
v_left  = v - ωL/2

ω_right = v_right/r
ω_left  = v_left/r
```

With the default `L=0.50 m`, `r=0.15 m`, `v=2.0 m/s`, and `ω=1.0 rad/s`:

```text
v_right = 2.25 m/s       ω_right = 15.00 rad/s
v_left  = 1.75 m/s       ω_left  = 11.67 rad/s
```

The Xacro wheel placement and Gazebo DiffDrive plugin read the same `track_width`
and `wheel_radius` values, preventing geometry/controller mismatch.

## Acceleration and braking

`velocity_limiter` applies the following update to linear and angular commands:

```text
v_next = v_current + clamp(
  v_target - v_current,
  -max_deceleration * dt,
   max_acceleration * dt
)
```

For the default linear limits:

```text
0 -> 5 m/s: 5 / 1.2 = 4.17 s
5 -> 0 m/s: 5 / 6.0 = 0.83 s
```

The limiter also bounds target velocity, brakes to zero before reversing, and
brakes automatically if target commands become stale.

## Test and validation

Run the complete workspace test suite:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Validate the expanded URDF separately:

```bash
check_urdf src/symbotic_description/urdf/naked/symbotic_diff_drive.urdf
```

The current automated suite covers command priority, keyboard mappings,
combined linear/angular commands, velocity limiting, braking, angle handling,
and Go-To-Goal control. The full suite currently contains 33 tests.

Useful runtime checks:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel/autonomous --once
ros2 topic echo /cmd_vel/target --once
ros2 topic echo /cmd_vel --once
```

## Troubleshooting

### Package not found

Source the workspace in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/Symbotic_test/install/setup.bash
```

### Go-To-Goal publishes feedback but the robot does not move

Confirm that `command_arbiter` is running. The action server intentionally
publishes to `/cmd_vel/autonomous`, not directly to Gazebo.

```bash
ros2 node list
ros2 topic echo /cmd_vel/autonomous --once
ros2 topic echo /cmd_vel/target --once
```

### Gazebo does not open

Verify that Gazebo Harmonic and `ros_gz` are installed, or run the server-only
simulation:

```bash
ros2 launch symbotic_description simulation.launch.py headless:=true
```

## Scope

Go-To-Goal is intentionally a simple odometry-based proportional controller.
It does not implement obstacle avoidance, global path planning, localization,
or SLAM. Those features are outside the requested project scope.

## License

Apache-2.0
