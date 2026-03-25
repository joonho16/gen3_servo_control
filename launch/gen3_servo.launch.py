"""
Launch file for Gen3 7DOF twist_controller based control with JointState goal interface.

Architecture:
  goal JointState -> bridge (Jacobian) -> Twist -> twist_controller -> Gen3 hardware
  Gen3 hardware -> joint_state_broadcaster -> /joint_states -> bridge -> current JointState

Usage:
  ros2 launch gen3_servo_control gen3_servo.launch.py robot_ip:=192.168.1.10
  ros2 launch gen3_servo_control gen3_servo.launch.py use_fake_hardware:=true control_hz:=200.0
  ros2 launch gen3_servo_control gen3_servo.launch.py control_hz:=50.0 gain:=0.5

Topics:
  Input:  /servo_joint_bridge/goal_joint_states  (sensor_msgs/JointState)
  Output: /servo_joint_bridge/current_joint_states (sensor_msgs/JointState)
          /joint_states (sensor_msgs/JointState) - raw from joint_state_broadcaster
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    input_mode = LaunchConfiguration('input_mode').perform(context)
    control_hz = float(LaunchConfiguration('control_hz').perform(context))
    gain = float(LaunchConfiguration('gain').perform(context))
    gain_angular = float(LaunchConfiguration('gain_angular').perform(context))
    max_joint_vel = float(LaunchConfiguration('max_joint_vel').perform(context))
    max_linear_vel = float(LaunchConfiguration('max_linear_vel').perform(context))
    max_angular_vel = float(LaunchConfiguration('max_angular_vel').perform(context))

    # Load MoveIt config (for Servo collision monitoring)
    moveit_config = MoveItConfigsBuilder(
        "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
    ).to_moveit_configs()

    # Load servo config and nest under 'moveit_servo' for servo_node_main
    servo_config_path = os.path.join(
        FindPackageShare('gen3_servo_control').perform(context),
        'config', 'servo_config.yaml'
    )
    with open(servo_config_path) as f:
        servo_yaml = yaml.safe_load(f)

    servo_params = {'moveit_servo': servo_yaml}

    # MoveIt Servo node (collision monitoring / safety)
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output='screen',
    )

    # Bridge node: goal JointState -> Jacobian -> Twist -> /twist_controller/commands
    bridge_node = Node(
        package='gen3_servo_control',
        executable='servo_joint_bridge',
        name='servo_joint_bridge',
        parameters=[{
            'input_mode': input_mode,
            'control_hz': control_hz,
            'gain': gain,
            'gain_angular': gain_angular,
            'max_joint_vel': max_joint_vel,
            'max_linear_vel': max_linear_vel,
            'max_angular_vel': max_angular_vel,
            'delta_threshold': 0.001,
            'pos_threshold': 0.001,
            'rot_threshold': 0.01,
            'base_link': 'base_link',
            'tip_link': 'tool_frame',
            'joint_names': [
                'joint_1', 'joint_2', 'joint_3', 'joint_4',
                'joint_5', 'joint_6', 'joint_7'
            ],
        }],
        output='screen',
    )

    return [servo_node, bridge_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.1.10',
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='Use fake hardware for testing'
        ),
        DeclareLaunchArgument(
            'gripper', default_value='robotiq_2f_85',
            description='Gripper type (empty string for no gripper)',
            choices=['', 'robotiq_2f_85', 'robotiq_2f_140'],
        ),
        DeclareLaunchArgument(
            'input_mode', default_value='joint',
            description="Input mode: 'joint' (JointState goal) or 'pose' (PoseStamped goal)"
        ),
        DeclareLaunchArgument(
            'control_hz', default_value='100.0',
            description='Bridge node control loop rate in Hz'
        ),
        DeclareLaunchArgument(
            'gain', default_value='1.0',
            description='Proportional gain (linear for pose mode, joint for joint mode)'
        ),
        DeclareLaunchArgument(
            'gain_angular', default_value='-1.0',
            description='Angular gain for pose mode (-1 = use gain value)'
        ),
        DeclareLaunchArgument(
            'max_joint_vel', default_value='0.5',
            description='Maximum joint velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'max_linear_vel', default_value='0.5',
            description='Maximum linear twist velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_vel', default_value='1.0',
            description='Maximum angular twist velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Launch RViz?'
        ),
    ]

    # Include kortex_control.launch.py directly (not gen3.launch.py)
    # so we can control both robot_controller and robot_pos_controller
    # to avoid double-spawning twist_controller
    kortex_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kortex_bringup'), 'launch', 'kortex_control.launch.py'
            ])
        ),
        launch_arguments={
            'robot_type': 'gen3',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'dof': '7',
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'gripper': LaunchConfiguration('gripper'),
            'gripper_joint_name': 'robotiq_85_left_knuckle_joint',
            'use_internal_bus_gripper_comm': 'false',
            'description_file': 'gen3.xacro',
            'controllers_file': 'ros2_controllers_parametric.yaml',
            # twist_controller = active, JTC = inactive (swapped)
            'robot_controller': 'twist_controller',
            'robot_pos_controller': 'joint_trajectory_controller',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items(),
    )

    # Include MoveIt move_group for planning scene (used by Servo)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kinova_gen3_7dof_robotiq_2f_85_moveit_config'),
                'launch', 'move_group.launch.py'
            ])
        ),
    )

    return LaunchDescription(
        declared_arguments + [
            kortex_launch,
            move_group_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
