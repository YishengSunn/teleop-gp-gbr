# Geo-GP Teleoperation (ROS 2 Humble)

This workspace integrates dual-arm teleoperation with a Geo-GP prediction pipeline.
The key goal is to use short human prompt motions to trigger GP-based trajectory prediction, then execute on the follower arm while keeping teleoperation behavior stable.

## Core Pipeline (GP on top of teleop)

1. `prompt_recorder` listens to follower EE state and publishes `PromptTrajectory` on `/gp_prompt_trajectory`.
2. `prediction_node` loads GP skills (6D models) and predicts `PredictedTrajectory` on `/gp_predicted_trajectory`.
3. `trajectory_executor` converts predicted trajectory into `/execution/desired_pose` and publishes `/execution/running`.
4. `cartesian_impedance_controller` tracks execution pose and can blend back to leader after execution.
5. During execution/blend, `prompt_recorder` is gated by state topics to avoid self-triggered re-recording loops.

## Main Packages

- `bimanual_architecture/geo_gp/geo_gp_prompt`: prompt capture (`prompt_recorder`)
- `bimanual_architecture/geo_gp/geo_gp_prediction`: GP inference (`prediction_node`, `predictor.py`)
- `bimanual_architecture/geo_gp/geo_gp_execution`: trajectory execution (`trajectory_executor`)
- `bimanual_architecture/geo_gp/geo_gp_controllers`: follower/leader controllers and broadcasters
- `bimanual_architecture/geo_gp/geo_gp_bringup`: launch and controller config

## Run

### 1) Bring up robot arms (leader/follower)

```bash
ros2 launch geo_gp_bringup single_arm.launch.py robot_ip:=<ip> ns:=leader arm_id:=leader
ros2 launch geo_gp_bringup single_arm.launch.py robot_ip:=<ip> ns:=follower arm_id:=follower control_mode:=cartesian_impedance
```

### 2) Start GP teleop pipeline

```bash
ros2 launch geo_gp_bringup geo_gp.launch.py
```

## Key Config

- `geo_gp_bringup/config/single_controllers.yaml`
  - follower `cartesian_impedance_controller` blend timing and state topics
  - leader `gravity_compensation_with_joint_torque_feedback_controller` feedback behavior
- `geo_gp_bringup/launch/geo_gp.launch.py`
  - prompt source topic (currently follower)
  - GP model directory
  - execution topics (`/execution/desired_pose`, `/execution/running`)

## Optional Tool

Record raw EE trajectory:

```bash
ros2 run trajectory_recorder record_ee_trajectory
```

Move to a Target Pose

```bash
ros2 launch teleop_csv_player move_to_pose.launch.py
```

Execute a CSV Trajectory

```bash
ros2 launch teleop_csv_player csv_player.launch.py
```
