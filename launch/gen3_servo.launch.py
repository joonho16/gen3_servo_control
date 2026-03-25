"""
Launch file for Gen3 7DOF servo control with JointState/Pose goal interface.
Includes the official kinova robot.launch.py and adds bridge + servo nodes.

Usage:
  ros2 launch gen3_servo_control gen3_servo.launch.py robot_ip:=192.168.50.10
  ros2 launch gen3_servo_control gen3_servo.launch.py robot_ip:=192.168.50.10 launch_rviz:=true

After launch, switch to twist_controller:
  ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
    activate_controllers: [twist_controller],
    deactivate_controllers: [joint_trajectory_controller],
    strictness: 1, activate_asap: true}"
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    control_hz = float(LaunchConfiguration("control_hz").perform(context))
    gain = float(LaunchConfiguration("gain").perform(context))
    gain_angular = float(LaunchConfiguration("gain_angular").perform(context))
    gain_d = float(LaunchConfiguration("gain_d").perform(context))
    gain_d_angular = float(LaunchConfiguration("gain_d_angular").perform(context))
    max_linear_vel = float(LaunchConfiguration("max_linear_vel").perform(context))
    max_angular_vel = float(LaunchConfiguration("max_angular_vel").perform(context))

    # Build MoveIt config just to get robot_description for bridge node
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "gripper": "robotiq_2f_85",
            "gripper_joint_name": "robotiq_85_left_knuckle_joint",
            "dof": "7",
            "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
        })
        .to_moveit_configs()
    )

    # ── MoveIt Servo (collision monitoring) ──
    servo_config_path = os.path.join(
        get_package_share_directory("gen3_servo_control"),
        "config", "servo_config.yaml",
    )
    with open(servo_config_path) as f:
        servo_yaml = yaml.safe_load(f)

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            {"moveit_servo": servo_yaml},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    # ── Bridge node ──
    bridge_node = Node(
        package="gen3_servo_control",
        executable="servo_joint_bridge",
        name="servo_joint_bridge",
        parameters=[
            moveit_config.robot_description,
            {
                "control_hz": control_hz,
                "gain": gain,
                "gain_angular": gain_angular,
                "gain_d": gain_d,
                "gain_d_angular": gain_d_angular,
                "max_linear_vel": max_linear_vel,
                "max_angular_vel": max_angular_vel,
                "pos_threshold": 0.001,
                "rot_threshold": 0.01,
                "base_link": "base_link",
                "tip_link": "end_effector_link",
            },
        ],
        output="screen",
    )

    switch_controller = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/controller_manager/switch_controller',
                'controller_manager_msgs/srv/SwitchController',
                '{activate_controllers: [twist_controller], '
                'deactivate_controllers: [joint_trajectory_controller], '
                'strictness: 1, activate_asap: true}',
            ],
            output='screen',
        )],
    )

    return [servo_node, bridge_node, switch_controller]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("robot_ip", description="IP address of the robot"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("use_internal_bus_gripper_comm", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("control_hz", default_value="100.0"),
        DeclareLaunchArgument("gain", default_value="2.0"),
        DeclareLaunchArgument("gain_angular", default_value="25.0"),
        DeclareLaunchArgument("gain_d", default_value="0.1"),
        DeclareLaunchArgument("gain_d_angular", default_value="5.0"),
        DeclareLaunchArgument("max_linear_vel", default_value="1.0"),
        DeclareLaunchArgument("max_angular_vel", default_value="30.0"),
    ]

    # Include the official Kinova robot.launch.py (proven to work)
    official_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
                "launch", "robot.launch.py",
            ])
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "use_internal_bus_gripper_comm": LaunchConfiguration("use_internal_bus_gripper_comm"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [
            official_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
