# gen3_servo_control

Kinova Gen3 7DOF real-time twist control package with JointState / PoseStamped goal interface.

## Architecture

```
[goal JointState] --> servo_joint_bridge --> [Twist] --> twist_controller --> Gen3 Hardware
[goal PoseStamped] --> servo_joint_bridge ----^                                   |
                            ^                                                     |
                            +--- [TF: base_link->end_effector_link] <-- joint_state_broadcaster
```

- **Joint mode**: goal JointState -> Jacobian -> base frame twist -> tool frame twist -> twist_controller
- **Pose mode**: goal PoseStamped -> Cartesian P-controller -> base frame twist -> tool frame twist -> twist_controller
- Current EE pose is read from **TF** (not FK)
- Twist is automatically transformed from base frame to tool frame (Kinova `CARTESIAN_REFERENCE_FRAME_TOOL`)
- MoveIt Servo runs alongside for collision monitoring

## Prerequisites

- [ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex) workspace built
- `ros-humble-moveit-servo` installed:
  ```bash
  sudo apt install ros-humble-moveit-servo
  ```

## Build

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select gen3_servo_control
source install/setup.bash
```

## Launch

```bash
# Basic launch
ros2 launch gen3_servo_control gen3_servo.launch.py robot_ip:=192.168.50.10

# Recommended safe starting values
ros2 launch gen3_servo_control gen3_servo.launch.py \
  robot_ip:=192.168.50.10 \
  gain:=0.3 \
  max_linear_vel:=0.05 \
  max_angular_vel:=0.2
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_ip` | (required) | Robot IP address |
| `use_fake_hardware` | `false` | Use mock hardware (twist_controller not available in sim) |
| `use_internal_bus_gripper_comm` | `true` | Use arm's internal gripper communication |
| `launch_rviz` | `true` | Launch RViz with MoveIt |
| `input_mode` | `joint` | Initial mode: `joint` or `pose` |
| `control_hz` | `100.0` | Bridge control loop rate (Hz) |
| `gain` | `1.0` | P-gain (position for pose mode, joint for joint mode) |
| `gain_angular` | `-1.0` | Angular P-gain for pose mode (`-1` = same as `gain`) |
| `max_joint_vel` | `0.5` | Max joint velocity in joint mode (rad/s) |
| `max_linear_vel` | `0.5` | Max Cartesian linear velocity (m/s) |
| `max_angular_vel` | `1.0` | Max Cartesian angular velocity (rad/s) |

## Usage

### 1. Switch to twist_controller

The robot starts with `joint_trajectory_controller` active. Switch to `twist_controller`:

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [twist_controller],
  deactivate_controllers: [joint_trajectory_controller],
  strictness: 1,
  activate_asap: true
}"
```

Verify:

```bash
ros2 control list_controllers
# twist_controller should be 'active'
```

### 2. Check current state

```bash
# Current joint positions
ros2 topic echo /servo_joint_bridge/current_joint_states --once

# Current EE pose (from TF)
ros2 topic echo /servo_joint_bridge/current_pose --once
```

### 3-A. Joint mode

Send a target joint configuration. Auto-switches to joint mode.

```bash
ros2 topic pub /servo_joint_bridge/goal_joint_states sensor_msgs/JointState \
  "{name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'],
    position: [0.0, -0.349, -3.1416, -2.548, 0.0, -0.873, 1.5708]}" -1
```

### 3-B. Pose mode

Send a target EE pose (XYZ + quaternion, `base_link` frame). Auto-switches to pose mode.

```bash
# Always check current pose first
ros2 topic echo /servo_joint_bridge/current_pose --once

# Send goal (small changes from current pose!)
ros2 topic pub /servo_joint_bridge/goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {
    position: {x: 0.122, y: 0.001, z: 0.358},
    orientation: {x: 0.707, y: 0.707, z: 0.025, w: 0.025}}}" -1
```

### 4. Switch back to joint_trajectory_controller

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [joint_trajectory_controller],
  deactivate_controllers: [twist_controller],
  strictness: 1,
  activate_asap: true
}"
```

## Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current robot joint states |
| `~/goal_joint_states` | `sensor_msgs/JointState` | Goal joint positions (switches to joint mode) |
| `~/goal_pose` | `geometry_msgs/PoseStamped` | Goal EE pose in `base_link` (switches to pose mode) |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/twist_controller/commands` | `geometry_msgs/Twist` | Twist command (tool frame) |
| `~/current_joint_states` | `sensor_msgs/JointState` | Current 7 arm joint states |
| `~/current_pose` | `geometry_msgs/PoseStamped` | Current EE pose from TF |

## Tuning

| Parameter | Effect |
|-----------|--------|
| `gain` | Higher = faster, lower = smoother. Start with `0.3` |
| `gain_angular` | Separate angular gain for pose mode. `-1` = use `gain` |
| `max_linear_vel` | Cartesian velocity limit. Start with `0.05` m/s |
| `max_angular_vel` | Angular velocity limit. Start with `0.2` rad/s |
| `max_joint_vel` | Joint velocity limit (joint mode). Default `0.5` rad/s |
| `control_hz` | Control loop rate. `100.0` is good for most cases |

## Notes

- `twist_controller` is **only available on real Kinova hardware** (not mock/Gazebo)
- The bridge auto-switches mode based on which goal topic receives a message
- Always check `current_pose` before sending a pose goal
- MoveIt Servo runs in parallel for collision monitoring
- `joint_trajectory_controller` can still be used via MoveIt when switched back
