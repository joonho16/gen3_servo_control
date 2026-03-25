#!/usr/bin/env python3
"""
Bridge node for Gen3 twist_controller with two input modes:

  Mode 'joint' (default):
    goal JointState -> Jacobian -> Twist -> twist_controller

  Mode 'pose':
    goal PoseStamped (XYZ + quat) -> Cartesian P-controller -> Twist -> twist_controller

Current EE pose is read from TF (base_link -> end_effector_link).
Twist is published in base_link frame (no tool frame transform - Kinova handles it).

Subscriptions:
  - /joint_states (sensor_msgs/JointState): current robot state
  - ~/goal_joint_states (sensor_msgs/JointState): goal joint positions (joint mode)
  - ~/goal_pose (geometry_msgs/PoseStamped): goal Cartesian pose (pose mode)

Publications:
  - /twist_controller/commands (geometry_msgs/Twist): twist to twist_controller
  - ~/current_joint_states (sensor_msgs/JointState): current state republish
  - ~/current_pose (geometry_msgs/PoseStamped): current EE pose from TF
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
import tf2_ros

import PyKDL
import numpy as np
from urdf_parser_py.urdf import URDF
import math


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
        origin = joint.origin
        if origin is None:
            frame = PyKDL.Frame.Identity()
        else:
            xyz = origin.xyz if origin.xyz else [0, 0, 0]
            rpy = origin.rpy if origin.rpy else [0, 0, 0]
            frame = PyKDL.Frame(
                PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
                PyKDL.Vector(xyz[0], xyz[1], xyz[2]))

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


def quat_error_to_angular_vel(q_goal, q_cur):
    """Compute angular velocity from quaternion error (in base frame)."""
    # q_error = q_goal * q_cur^-1
    # q_cur^-1 (conjugate for unit quaternion)
    q_cur_inv = [-q_cur[0], -q_cur[1], -q_cur[2], q_cur[3]]

    # Hamilton product: q_goal * q_cur_inv
    gx, gy, gz, gw = q_goal
    cx, cy, cz, cw = q_cur_inv
    ew = gw*cw - gx*cx - gy*cy - gz*cz
    ex = gw*cx + gx*cw + gy*cz - gz*cy
    ey = gw*cy - gx*cz + gy*cw + gz*cx
    ez = gw*cz + gx*cy - gy*cx + gz*cw

    # Ensure shortest path
    if ew < 0:
        ex, ey, ez, ew = -ex, -ey, -ez, -ew

    # Axis-angle from quaternion: angle = 2*acos(w), axis = [x,y,z]/sin(angle/2)
    sin_half = math.sqrt(ex*ex + ey*ey + ez*ez)
    if sin_half < 1e-6:
        return np.zeros(3)

    angle = 2.0 * math.atan2(sin_half, ew)
    axis = np.array([ex, ey, ez]) / sin_half
    return axis * angle


def rotate_vec_by_quat(v, qx, qy, qz, qw):
    """Rotate vector v by quaternion (qx,qy,qz,qw). Returns rotated vector."""
    # q * v * q^-1 using Hamilton product
    # v as pure quaternion (vx, vy, vz, 0)
    q = np.array([qw, qx, qy, qz])
    v_quat = np.array([0.0, v[0], v[1], v[2]])

    # Hamilton product: q * v_quat
    def ham(a, b):
        return np.array([
            a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])

    q_conj = np.array([qw, -qx, -qy, -qz])
    result = ham(ham(q, v_quat), q_conj)
    return result[1:4]


class ServoJointBridge(Node):
    def __init__(self):
        super().__init__('servo_joint_bridge')

        self.declare_parameter('robot_description', '')
        self.declare_parameter('control_hz', 100.0)
        self.declare_parameter('input_mode', 'joint')
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('gain_angular', -1.0)
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
        self.declare_parameter('tip_link', 'end_effector_link')

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
        self.goal_pose = None

        # TF2 for current EE pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # KDL chain for Jacobian (joint mode only)
        self.chain = None
        self.jac_solver = None
        self.kdl_joint_names = []
        robot_description = self.get_parameter('robot_description').value
        if robot_description:
            self._build_kdl_chain(robot_description)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)
        self.goal_joint_sub = self.create_subscription(
            JointState, '~/goal_joint_states', self.goal_joint_cb, 10)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '~/goal_pose', self.goal_pose_cb, 10)

        # Publishers
        self.twist_pub = self.create_publisher(
            Twist, '/twist_controller/commands', 10)
        self.current_joint_pub = self.create_publisher(
            JointState, '~/current_joint_states', 10)
        self.current_pose_pub = self.create_publisher(
            PoseStamped, '~/current_pose', 10)

        # Control loop
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f'ServoJointBridge started: mode={self.input_mode}, '
            f'hz={self.control_hz}, gain={self.gain}, '
            f'gain_angular={self.gain_angular}')

    def _build_kdl_chain(self, urdf_string):
        try:
            urdf_model = URDF.from_xml_string(urdf_string)
            self.chain = build_kdl_chain(urdf_model, self.base_link, self.tip_link)
            self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)
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

    def _get_current_ee_pose(self):
        """Get current EE pose from TF. Returns (pos, quat) or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_link, self.tip_link, rclpy.time.Time())
            pos = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z])
            quat = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w]
            return pos, quat
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    # ── Callbacks ─────────────────────────────────────────────────────

    def joint_state_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_positions[name] = msg.position[i]

        out = JointState()
        out.header = msg.header
        for name in self.joint_names:
            if name in self.current_positions:
                out.name.append(name)
                out.position.append(self.current_positions[name])
        self.current_joint_pub.publish(out)

        # Publish current EE pose from TF
        result = self._get_current_ee_pose()
        if result is not None:
            pos, quat = result
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.base_link
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            self.current_pose_pub.publish(pose)

    def goal_joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                self.goal_positions[name] = msg.position[i]
        self.input_mode = 'joint'

    def goal_pose_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        self.input_mode = 'pose'

    # ── Control loop ──────────────────────────────────────────────────

    def control_loop(self):
        if self.input_mode == 'joint':
            self._control_joint_mode()
        elif self.input_mode == 'pose':
            self._control_pose_mode()

    def _control_joint_mode(self):
        """Joint goal -> Jacobian -> Twist (in base frame)."""
        if not self.current_positions or not self.goal_positions:
            return
        if self.jac_solver is None or self.chain is None:
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

        jacobian = PyKDL.Jacobian(n_kdl)
        self.jac_solver.JntToJac(q_current, jacobian)

        J = np.zeros((6, n_kdl))
        for i in range(6):
            for j in range(n_kdl):
                J[i, j] = jacobian[i, j]

        twist_base = J @ q_vel

        # Transform base frame twist -> tool frame twist
        result = self._get_current_ee_pose()
        if result is not None:
            _, cur_quat = result
            qx, qy, qz, qw = cur_quat
            lin_tool = rotate_vec_by_quat(twist_base[:3], -qx, -qy, -qz, qw)
            ang_tool = rotate_vec_by_quat(twist_base[3:], -qx, -qy, -qz, qw)
            twist_vec = np.concatenate([lin_tool, ang_tool])
        else:
            twist_vec = twist_base

        self._publish_clamped_twist(twist_vec)

    def _control_pose_mode(self):
        """Pose goal -> Cartesian P-controller -> Twist (in base frame)."""
        if self.goal_pose is None:
            return

        result = self._get_current_ee_pose()
        if result is None:
            return

        cur_pos, cur_quat = result

        gp = self.goal_pose.pose
        goal_pos = np.array([gp.position.x, gp.position.y, gp.position.z])
        goal_quat = [gp.orientation.x, gp.orientation.y,
                     gp.orientation.z, gp.orientation.w]

        # Position error (base frame)
        pos_error = goal_pos - cur_pos
        pos_err_norm = np.linalg.norm(pos_error)

        # Orientation error (base frame)
        rot_err_vec = quat_error_to_angular_vel(goal_quat, cur_quat)
        rot_err_norm = np.linalg.norm(rot_err_vec)

        if pos_err_norm < self.pos_threshold and rot_err_norm < self.rot_threshold:
            self.twist_pub.publish(Twist())
            return

        # P-controller (base frame)
        lin_vel_base = self.gain * pos_error
        ang_vel_base = self.gain_angular * rot_err_vec

        # Transform base frame twist -> tool frame twist
        # TF quat (base->ee) rotates ee->base, so inverse (conjugate) = base->ee
        qx, qy, qz, qw = cur_quat
        # Conjugate = inverse for unit quaternion: rotates base->tool
        lin_vel_tool = rotate_vec_by_quat(lin_vel_base, -qx, -qy, -qz, qw)
        ang_vel_tool = rotate_vec_by_quat(ang_vel_base, -qx, -qy, -qz, qw)

        twist_vec = np.concatenate([lin_vel_tool, ang_vel_tool])
        self._publish_clamped_twist(twist_vec)

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
