# diff_drive_robot

ROS 2 bringup package for the complete differential-drive robot backend.

```bash
ros2 launch diff_drive_robot demo.launch.py
```

The launch starts Gazebo, robot state publishing, the ROS-Gazebo bridge,
velocity limiting, command arbitration, and the Go-To-Goal action server.
Keyboard teleoperation remains a separate process because it needs direct
terminal input.
