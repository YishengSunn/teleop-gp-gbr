# Teleoperation Project (ROS 2)

This project is built on top of **multipanda_ros2** and is currently used for **robot teleoperation** with support for both **leader** and **follower** arms. It also provides tools for **trajectory recording** on real hardware and **trajectory execution** using Cartesian impedance control.

## Launch Teleoperation

You can start either a **leader** or a **follower** arm using the same launch file by specifying the namespace (`ns`) and `arm_id`.

Example:

```bash
ros2 launch teleop_bringup single_arm.launch.py \
  robot_ip:=192.168.3.101 \
  ns:=follower \
  arm_id:=follower
```

### Parameters

- `robot_ip` — IP address of the robot controller
- `ns` — ROS namespace (`leader` or `follower`)
- `arm_id` — Robot arm identifier (`leader` or `follower`)
- `control_mode` — Control mode (recommended: `cartesian_impedance`)

## Record End-Effector Trajectory (Real Robot)

To record end-effector trajectory data from the physical robot:

```bash
ros2 run trajectory_recorder record_ee_trajectory
```

This will collect real robot motion data for later playback or analysis.

## Move to a Target Pose

When using **Cartesian impedance control mode**, you can command the robot to move to a target pose.  
The system will interpolate from the current pose to the target pose.

```bash
ros2 launch teleop_csv_player move_to_pose.launch.py
```

## Execute a CSV Trajectory

Also under **Cartesian impedance control mode**, you can execute a trajectory stored in a CSV file:

```bash
ros2 launch teleop_csv_player csv_player.launch.py
```

This will replay the trajectory on the robot.
