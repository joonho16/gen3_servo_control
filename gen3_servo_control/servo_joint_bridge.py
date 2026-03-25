#!/usr/bin/env python3
"""
Bridge node for Gen3 twist_controller with two input modes:

  Mode 'joint' (default):
    goal JointState -> Jacobian -> Twist -> twist_controller

  Mode 'pose':
    goal PoseStamped (XYZ + quaternion) -> Cartesian P-controller -> Twist -> twist_controller

Both modes publish current robot state as JointState.

Subscriptions:
  - /joint_states (sensor_msgs/JointState): current robot state
  - ~/goal_joint_states (sensor_msgs/JointState): goal joint positions (joint mode)
  - ~/goal_pose (geometry_msgs/PoseStamped): goal Cartesian pose (pose mode)

Publications:
  - /twist_controller/commands (geometry_msgs/Twist): twist to twist_controller
  - ~/current_joint_states (sensor_msgs/JointState): current state republish
  - ~/current_pose (geometry_msgs/PoseStamped): current EE pose (always published)

Parameters:
  - control_hz (double): control loop rate in Hz (default: 100.0)
  - input_mode (string): 'joint' or 'pose' (default: 'joint')
  - gain (double): proportional gain (default: 1.0)
  - gain_angular (double): angular gain for pose mode (default: 1.0, uses 'gain' if not set)
  - max_joint_vel (double): max joint velocity for joint mode (default: 0.5 rad/s)
  - max_linear_vel (double): max Cartesian linear velocity (default: 0.5 m/s)
  - max_angular_vel (double): max Cartesian angular velocity (default: 1.0 rad/s)
  - pos_threshold (double): position error threshold for pose mode (default: 0.001 m)
  - rot_threshold (double): rotation error threshold for pose mode (default: 0.01 rad)
  - delta_threshold (double): joint error threshold for joint mode (default: 0.001 rad)
  - base_link (string): base frame (default: 'base_link')
  - tip_link (string): EE frame (default: 'tool_frame')
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

import PyKDL
import numpy as np
from urdf_parser_py.urdf import URDF


def kdl_frame_from_urdf_origin(origin):
    if origin is None:
        return PyKDL.Frame.Identity()
    xyz = origin.xyz if origin.xyz else [0, 0, 0]
    rpy = origin.rpy if origin.rpy else [0, 0, 0]
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
        PyKDL.Vector(xyz[0], xyz[1], xyz[2])
    )


def build_kdl_chain(urdf_model, base_link, tip_link):
    """Build a KDL chain from URDF model between base_link and tip_link."""
    chain = PyKDL.Chain()
    link_name = tip_link
    links = []
    while link_name != base_link:
        parent_joint = None
        for jnt in urdf_model.joints:
            if jnt.child == link_name:
                parent_joint = jnt
                break
        if parent_joint is None:
            raise RuntimeError(f"Cannot find parent joint for link '{link_name}'")
        links.append((parent_joint, link_name))
        link_name = parent_joint.parent

    links.reverse()

    for joint, child_link in links:
        frame = kdl_frame_from_urdf_origin(joint.origin)
        axis = joint.axis if joint.axis else [0, 0, 1]

        if joint.type in ('revolute', 'continuous'):
            kdl_joint = PyKDL.Joint(
                joint.name, PyKDL.Vector(0, 0, 0),
                PyKDL.Vector(axis[0], axis[1], axis[2]),
                PyKDL.Joint.RotAxis)
        elif joint.type == 'prismatic':
            kdl_joint = PyKDL.Joint(
                joint.name, PyKDL.Vector(0, 0, 0),
                PyKDL.Vector(axis[0], axis[1], axis[2]),
                PyKDL.Joint.TransAxis)
        else:
            kdl_joint = PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)

        chain.addSegment(PyKDL.Segment(child_link, kdl_joint, frame))

    return chain


def quat_to_kdl_rotation(x, y, z, w):
    return PyKDL.Rotation.Quaternion(x, y, z, w)


def kdl_rotation_to_axis_angle(rot):
    """Extract axis-angle from KDL Rotation. Returns (axis, angle)."""
    angle = rot.GetRotAngle()
    # GetRotAngle returns (Vector axis, double angle)
    return angle


class ServoJointBridge(Node):
    def __init__(self):
        super().__init__('servo_joint_bridge')

        # Declare all parameters
        self.declare_parameter('control_hz', 100.0)
        self.declare_parameter('input_mode', 'joint')  # 'joint' or 'pose'
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('gain_angular', -1.0)  # -1 means use 'gain'
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3', 'joint_4',
            'joint_5', 'joint_6', 'joint_7'
        ])
        self.declare_parameter('delta_threshold', 0.001)
        self.declare_parameter('pos_threshold', 0.001)
        self.declare_parameter('rot_threshold', 0.01)
        self.declare_parameter('max_joint_vel', 0.5)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'tool_frame')

        # Read parameters
        self.control_hz = self.get_parameter('control_hz').value
        self.input_mode = self.get_parameter('input_mode').value
        self.gain = self.get_parameter('gain').value
        gain_ang = self.get_parameter('gain_angular').value
        self.gain_angular = gain_ang if gain_ang > 0 else self.gain
        self.joint_names = self.get_parameter('joint_names').value
        self.delta_threshold = self.get_parameter('delta_threshold').value
        self.pos_threshold = self.get_parameter('pos_threshold').value
        self.rot_threshold = self.get_parameter('rot_threshold').value
        self.max_joint_vel = self.get_parameter('max_joint_vel').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.base_link = self.get_parameter('base_link').value
        self.tip_link = self.get_parameter('tip_link').value

        self.current_positions = {}
        self.goal_positions = {}
        self.goal_pose = None  # PoseStamped for pose mode
        self.chain = None
        self.jac_solver = None
        self.fk_solver = None
        self.kdl_joint_names = []

        # Subscribe to robot_description to build KDL chain
        self.robot_desc_sub = self.create_subscription(
            String, '/robot_description', self.robot_description_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        # Subscribe to current joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)

        # Subscribe to goal joint states (always, for both modes)
        self.goal_joint_sub = self.create_subscription(
            JointState, '~/goal_joint_states', self.goal_joint_cb, 10)

        # Subscribe to goal pose (always, for both modes)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '~/goal_pose', self.goal_pose_cb, 10)

        # Publish twist to twist_controller
        self.twist_pub = self.create_publisher(
            Twist, '/twist_controller/commands', 10)

        # Republish current joint states
        self.current_joint_pub = self.create_publisher(
            JointState, '~/current_joint_states', 10)

        # Publish current EE pose
        self.current_pose_pub = self.create_publisher(
            PoseStamped, '~/current_pose', 10)

        # Control loop timer
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f'ServoJointBridge started: mode={self.input_mode}, '
            f'hz={self.control_hz}, gain={self.gain}, '
            f'gain_angular={self.gain_angular}')

    # ── URDF / KDL setup ──────────────────────────────────────────────

    def robot_description_cb(self, msg: String):
        if self.chain is not None:
            return
        try:
            urdf_model = URDF.from_xml_string(msg.data)
            self.chain = build_kdl_chain(urdf_model, self.base_link, self.tip_link)
            self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)
            self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)

            self.kdl_joint_names = []
            for i in range(self.chain.getNrOfSegments()):
                seg = self.chain.getSegment(i)
                jnt = seg.getJoint()
                if jnt.getType() != PyKDL.Joint.Fixed:
                    self.kdl_joint_names.append(jnt.getName())

            self.get_logger().info(
                f'KDL chain built: {self.base_link} -> {self.tip_link}, '
                f'{self.chain.getNrOfJoints()} joints: {self.kdl_joint_names}')
        except Exception as e:
            self.get_logger().error(f'Failed to build KDL chain: {e}')

    # ── Subscriber callbacks ──────────────────────────────────────────

    def joint_state_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_positions[name] = msg.position[i]

        # Republish filtered current joint state
        out = JointState()
        out.header = msg.header
        for name in self.joint_names:
            if name in self.current_positions:
                out.name.append(name)
                out.position.append(self.current_positions[name])
        self.current_joint_pub.publish(out)

        # Publish current EE pose via FK
        self._publish_current_pose(msg.header.stamp)

    def goal_joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                self.goal_positions[name] = msg.position[i]
        # Switch to joint mode on receiving joint goal
        self.input_mode = 'joint'

    def goal_pose_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        # Switch to pose mode on receiving pose goal
        self.input_mode = 'pose'

    # ── FK: publish current EE pose ───────────────────────────────────

    def _get_current_kdl_frame(self):
        """Compute FK for current joint positions. Returns KDL Frame or None."""
        if self.fk_solver is None or self.chain is None:
            return None
        n_kdl = self.chain.getNrOfJoints()
        q = PyKDL.JntArray(n_kdl)
        for i, name in enumerate(self.kdl_joint_names):
            if name not in self.current_positions:
                return None
            q[i] = self.current_positions[name]
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(q, frame)
        return frame

    def _publish_current_pose(self, stamp):
        frame = self._get_current_kdl_frame()
        if frame is None:
            return
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_link
        pose.pose.position.x = frame.p.x()
        pose.pose.position.y = frame.p.y()
        pose.pose.position.z = frame.p.z()
        qx, qy, qz, qw = frame.M.GetQuaternion()
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.current_pose_pub.publish(pose)

    # ── Control loop ──────────────────────────────────────────────────

    def control_loop(self):
        if self.chain is None:
            return

        if self.input_mode == 'joint':
            self._control_joint_mode()
        elif self.input_mode == 'pose':
            self._control_pose_mode()

    def _control_joint_mode(self):
        """Joint goal -> Jacobian -> Twist."""
        if not self.current_positions or not self.goal_positions:
            return
        if self.jac_solver is None:
            return

        n_kdl = self.chain.getNrOfJoints()
        q_current = PyKDL.JntArray(n_kdl)
        q_vel = np.zeros(n_kdl)
        has_motion = False

        for i, name in enumerate(self.kdl_joint_names):
            if name not in self.current_positions or name not in self.goal_positions:
                return
            cur = self.current_positions[name]
            goal = self.goal_positions[name]
            q_current[i] = cur
            error = goal - cur
            if abs(error) > self.delta_threshold:
                q_vel[i] = np.clip(self.gain * error,
                                   -self.max_joint_vel, self.max_joint_vel)
                has_motion = True

        if not has_motion:
            self.twist_pub.publish(Twist())
            return

        # Jacobian -> twist
        jacobian = PyKDL.Jacobian(n_kdl)
        self.jac_solver.JntToJac(q_current, jacobian)

        J = np.zeros((6, n_kdl))
        for i in range(6):
            for j in range(n_kdl):
                J[i, j] = jacobian[i, j]

        twist_vec = J @ q_vel
        self._publish_clamped_twist(twist_vec)

    def _control_pose_mode(self):
        """Pose goal (XYZ + quat) -> Cartesian P-controller -> Twist."""
        if self.goal_pose is None:
            return

        current_frame = self._get_current_kdl_frame()
        if current_frame is None:
            return

        # Goal pose
        gp = self.goal_pose.pose
        goal_pos = np.array([gp.position.x, gp.position.y, gp.position.z])
        goal_rot = quat_to_kdl_rotation(
            gp.orientation.x, gp.orientation.y,
            gp.orientation.z, gp.orientation.w)

        # Current pose
        cur_pos = np.array([current_frame.p.x(),
                            current_frame.p.y(),
                            current_frame.p.z()])

        # Position error
        pos_error = goal_pos - cur_pos
        pos_err_norm = np.linalg.norm(pos_error)

        # Orientation error: R_error = R_goal * R_current^T
        rot_error = goal_rot * current_frame.M.Inverse()
        angle_axis = rot_error.GetRotAngle()
        angle = angle_axis[0]  # scalar angle
        axis = angle_axis[1]   # KDL.Vector axis
        rot_err_vec = np.array([axis.x(), axis.y(), axis.z()]) * angle

        rot_err_norm = abs(angle)

        # Check thresholds
        if pos_err_norm < self.pos_threshold and rot_err_norm < self.rot_threshold:
            self.twist_pub.publish(Twist())
            return

        # P-controller
        lin_vel = self.gain * pos_error
        ang_vel = self.gain_angular * rot_err_vec

        twist_vec = np.concatenate([lin_vel, ang_vel])
        self._publish_clamped_twist(twist_vec)

    # ── Twist publish with clamping ───────────────────────────────────

    def _publish_clamped_twist(self, twist_vec):
        lin = twist_vec[:3].copy()
        ang = twist_vec[3:].copy()

        lin_norm = np.linalg.norm(lin)
        ang_norm = np.linalg.norm(ang)
        if lin_norm > self.max_linear_vel:
            lin *= self.max_linear_vel / lin_norm
        if ang_norm > self.max_angular_vel:
            ang *= self.max_angular_vel / ang_norm

        msg = Twist()
        msg.linear.x = float(lin[0])
        msg.linear.y = float(lin[1])
        msg.linear.z = float(lin[2])
        msg.angular.x = float(ang[0])
        msg.angular.y = float(ang[1])
        msg.angular.z = float(ang[2])
        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoJointBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
