# teleop_bringup

Bringup package for **Garmi bimanual teleoperation** (two Panda arms + base/head), supporting:

- **Real robot bringup** (Garmi hardware)
- **Simulation bringup** (MuJoCo)

This README explains how to create a ROS 2 workspace, add required repositories, build in Release mode (tests off), and run the provided launch files. It also documents the available launch arguments and their defaults based on the launch files.

---

## Workspace setup

Create a ROS 2 workspace and clone the required repositories into `src/`:

```bash
mkdir -p ~/teleop_ws/src
cd ~/teleop_ws/src

# 1) bringup repo
git clone -b panda https://gitlab.lrz.de/mohatorky/teleop_bringup.git

# 2) controllers repo
git clone -b panda https://gitlab.lrz.de/mohatorky/teleop_controllers.git
```

---

## Install multipanda_ros2 (required)

This stack depends on the multipanda/Garmi ROS 2 integration. Follow the upstream instructions:

- **multipanda_ros2 (instructions):**  
  [https://gitlab.lrz.de/geriatronics-project-y/garmi-development/multipanda_ros2.git](https://gitlab.lrz.de/geriatronics-project-y/garmi-development/multipanda_ros2.git)

Typical outcomes of following the multipanda instructions include installing system dependencies, setting up robot descriptions, ros2_control hardware interfaces, and potentially additional udev / realtime / networking requirements.


---

## Build (Release + testing OFF)

From the workspace root:

```bash
cd ~/teleop_ws

# Install dependencies for everything in src/
rosdep install --from-paths src --ignore-src -r -y

# Build optimized, with tests disabled
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

---

## Run

Source the workspace:

```bash
source ~/teleop_ws/install/setup.bash
```

### Real (hardware)

```bash
ros2 launch teleop_bringup real_garmi_teleop.launch.py
```

### Simulation (MuJoCo)

```bash
ros2 launch teleop_bringup sim_garmi_teleop.launch.py
```

---

## Real bringup: launch arguments (options + defaults)

Launch file: `teleop_bringup/launch/real/real_garmi_teleop.launch.py`

This launch uses a **fixed** main namespace:

- Namespace is hard-coded to: `garmi`

### Arguments

- `robot_ip_1` (default: `192.168.3.101`)  
  Hostname/IP of robot 1.

- `robot_ip_2` (default: `192.168.3.102`)  
  Hostname/IP of robot 2.

- `arm_id_1` (default: `left`)  
  Unique arm ID for robot 1.

- `arm_id_2` (default: `right`)  
  Unique arm ID for robot 2.

- `use_rviz` (default: `false`)  
  If `true`, starts RViz with `garmi_description/rviz/visualize_garmi.rviz`.

- `use_fake_hardware` (default: `false`)  
  If `true`, runs ros2_control with fake hardware.

- `fake_sensor_commands` (default: `false`)  
  Only valid when `use_fake_hardware=true`. Used to simulate sensor command interfaces.

- `load_gripper_1` (default: `false`)  
  Use Franka gripper as end-effector for robot 1.

- `load_gripper_2` (default: `false`)  
  Use Franka gripper as end-effector for robot 2.

- `control_mode` (**required**, no default)  
  Must be one of:
  - `cartesian_impedance`
  - `joint_impedance`
  - `cartesian_delta_impedance`

  The launch file validates this argument and will throw an error if it is missing or invalid.

### Controller configuration used

- Combined controllers YAML:  
  `teleop_bringup/config/real/garmi_teleop.yaml`

### What gets launched (high level)

Under namespace `/garmi`:

- `robot_state_publisher` with a robot description built from:  
  `garmi_description/robots/garmi_real.urdf.xacro`  
  (xacro is parameterized by IPs, arm IDs, gripper flags, and fake hardware flags)

- `joint_state_publisher` aggregating `/garmi/joint_states` (+ gripper joint states if enabled)

- One **shared** `controller_manager` (`ros2_control_node`) for all interfaces (arms + base + head)

- Spawners:
  - `joint_state_broadcaster` (with an explicit joints list including both arms + head + base joints)
  - Arm state + model broadcasters (only when `use_fake_hardware=false`):
    - `left_arm_state`, `left_arm_model`
    - `right_arm_state`, `right_arm_model`
  - Base controller:
    - `garmi_base_controller`
  - Optional impedance controllers, depending on `control_mode`:
    - `right_joint_impedance_controller`, `left_joint_impedance_controller`
    - `right_cartesian_impedance_controller`, `left_cartesian_impedance_controller`
    - `right_cartesian_delta_impedance_controller`, `left_cartesian_delta_impedance_controller`

- Optional RViz (if `use_rviz=true`)

- Qb hands control launch is included:
  - `garmi_hand_control/launch/garmi_qb_hands.launch.py` with `namespace:=garmi`

- Franka gripper launch includes:
  - `franka_gripper/launch/gripper.launch.py` for each robot IP, conditional on gripper enable flags.


---

## Simulation bringup: launch arguments (options + defaults)

Launch file: `teleop_bringup/launch/sim/sim_garmi_teleop.launch.py`

This launch uses a **fixed** namespace:

- Namespace is hard-coded to: `garmi`  
  (and must match the namespace used inside the MuJoCo ros2_control plugin config)

### Arguments

- `use_rviz` (default: `false`)  
  If `true`, RViz can be enabled by downstream launch components (this file itself sets up description + joint state publisher + MuJoCo server).

- `initial_positions_1` (default: `"-1.571 -0.785 0.0 -2.356 0.0 1.571 0.785"`)  
  Initial joint positions for robot 1. Must be enclosed in quotes and contain only numbers.  
  Default corresponds to the `"communication_test"` pose.

- `initial_positions_2` (default: `"1.571 -0.785 0.0 -2.356 0.0 1.571 0.785"`)  
  Initial joint positions for robot 2. Must be enclosed in quotes and contain only numbers.  
  Default corresponds to the `"communication_test"` pose.

### Scene / plugin configuration (as coded)

The sim launch currently has `load_gripper=False` and `load_lidars=False` hard-coded. With these fixed values, it uses:

- MuJoCo scene: `garmi_ng.xml`
- MuJoCo plugin YAML: `teleop_bringup/config/sim/sim_garmi_teleop.yaml`

### What gets launched (high level)

Under namespace `/garmi`:

- `robot_state_publisher` using xacro:  
  `garmi_description/robots/garmi_sim.urdf.xacro`  
  parameterized by arm IDs, gripper flags, and the initial positions

- `joint_state_publisher` aggregating `/garmi/joint_states` (+ gripper joint states if enabled)

- MuJoCo ROS server launch included:
  - `teleop_bringup/launch/sim/launch_mujoco_ros_server_ns.launch`
  - with `ns:=garmi`, `use_sim_time:=true`, and plugin config pointing to the YAML above

- Spawners (controller_manager within `/garmi/controller_manager`):
  - `joint_state_broadcaster`
  - `left_state_broadcaster`
  - `right_state_broadcaster`
  - `garmi_head_controller`
  - `garmi_base_controller`
  - `left_move_to_start_controller`
  - `right_move_to_start_controller`

---

## Examples

### Real: cartesian impedance + RViz

```bash
source ~/teleop_ws/install/setup.bash

ros2 launch teleop_bringup real_garmi_teleop.launch.py   control_mode:=cartesian_impedance   use_rviz:=true
```



### Sim: custom initial joint positions

```bash
ros2 launch teleop_bringup sim_garmi_teleop.launch.py   initial_positions_1:="\"-1.57 -0.50 0.0 -2.0 0.0 1.57 0.8\""   initial_positions_2:="\"1.57 -0.50 0.0 -2.0 0.0 1.57 0.8\""
```

(Notice the quotes: the launch argument expects the joint list string to be quoted.)

---

## Troubleshooting

- **Launch fails immediately with “Invalid control_mode”**
  - Make sure you set `control_mode:=...` and that it’s one of:
    `cartesian_impedance`, `joint_impedance`, `cartesian_delta_impedance`.

- **Controllers fail to spawn**
  - Check `teleop_bringup/config/real/garmi_teleop.yaml` (real) or the sim plugin YAML.
<!-- 
- **Grippers**
  - Ensure the upstream gripper setup is done.
  - Note the gripper condition issue mentioned above if robot 2 gripper doesn’t start. -->

---
