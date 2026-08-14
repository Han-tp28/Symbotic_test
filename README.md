# Symbotic Differential-Drive Simulation

ROS 2 Jazzy and Gazebo Harmonic simulation project for a configurable
differential-drive mobile robot.

## Development status

The repository is being implemented incrementally. The current milestone is
the ROS 2 workspace and package scaffold.

## Target environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / Gz-Sim 8
- `ros_gz`
- `colcon`

## Initial build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
