#!/usr/bin/env python3
"""
Bridge node for Gen3 twist_controller.

Inputs:
  - ~/goal_joint_states (sensor_msgs/JointState):
      position[0..6] = [dx, dy, dz, dqx, dqy, dqz, dqw] (cspace delta)
      현재 EE pose에 delta를 더해 새 goal로 설정
  - ~/goal_pose (geometry_msgs/PoseStamped):
      절대 목표 pose

Control: Cartesian PD -> Twist (tool frame) -> twist_controller
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros

import numpy as np
import math


def quat_error_to_angular_vel(q_goal, q_cur):
    q_cur_inv = [-q_cur[0], -q_cur[1], -q_cur[2], q_cur[3]]
    gx, gy, gz, gw = q_goal
    cx, cy, cz, cw = q_cur_inv
    ew = gw*cw - gx*cx - gy*cy - gz*cz
    ex = gw*cx + gx*cw + gy*cz - gz*cy
    ey = gw*cy - gx*cz + gy*cw + gz*cx
    ez = gw*cz + gx*cy - gy*cx + gz*cw
    if ew < 0:
        ex, ey, ez, ew = -ex, -ey, -ez, -ew
    sin_half = math.sqrt(ex*ex + ey*ey + ez*ez)
    if sin_half < 1e-6:
        return np.zeros(3)
    angle = 2.0 * math.atan2(sin_half, ew)
    return np.array([ex, ey, ez]) / sin_half * angle


def rotate_vec_by_quat(v, qx, qy, qz, qw):
    q = np.array([qw, qx, qy, qz])
    v_quat = np.array([0.0, v[0], v[1], v[2]])

    def ham(a, b):
        return np.array([
            a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])

    q_conj = np.array([qw, -qx, -qy, -qz])
    return ham(ham(q, v_quat), q_conj)[1:4]


class ServoJointBridge(Node):
    def __init__(self):
        super().__init__('servo_joint_bridge')

        self.declare_parameter('control_hz', 100.0)
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('gain_angular', -1.0)
        self.declare_parameter('gain_d', 0.0)
        self.declare_parameter('gain_d_angular', -1.0)
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3', 'joint_4',
            'joint_5', 'joint_6', 'joint_7'
        ])
        self.declare_parameter('pos_threshold', 0.001)
        self.declare_parameter('rot_threshold', 0.01)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'end_effector_link')

        self.control_hz = self.get_parameter('control_hz').value
        self.gain = self.get_parameter('gain').value
        gain_ang = self.get_parameter('gain_angular').value
        self.gain_angular = gain_ang if gain_ang > 0 else self.gain
        self.gain_d = self.get_parameter('gain_d').value
        gain_d_ang = self.get_parameter('gain_d_angular').value
        self.gain_d_angular = gain_d_ang if gain_d_ang >= 0 else self.gain_d
        self.joint_names = self.get_parameter('joint_names').value
        self.pos_threshold = self.get_parameter('pos_threshold').value
        self.rot_threshold = self.get_parameter('rot_threshold').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.base_link = self.get_parameter('base_link').value
        self.tip_link = self.get_parameter('tip_link').value

        self.current_positions = {}
        self.goal_pose = None
        self._prev_pos_error = np.zeros(3)
        self._prev_rot_err = np.zeros(3)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.add_on_set_parameters_callback(self._on_params_changed)

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)
        self.goal_joint_sub = self.create_subscription(
            JointState, '~/goal_joint_states', self.goal_joint_cb, 10)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '~/goal_pose', self.goal_pose_cb, 10)

        self.twist_pub = self.create_publisher(
            Twist, '/twist_controller/commands', 10)
        self.current_joint_pub = self.create_publisher(
            JointState, '~/current_joint_states', 10)
        self.current_pose_pub = self.create_publisher(
            PoseStamped, '~/current_pose', 10)

        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f'ServoJointBridge started: hz={self.control_hz}, '
            f'gain={self.gain}, gain_angular={self.gain_angular}')

    def _on_params_changed(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'gain':
                self.gain = p.value
            elif p.name == 'gain_angular':
                self.gain_angular = p.value if p.value > 0 else self.gain
            elif p.name == 'gain_d':
                self.gain_d = p.value
            elif p.name == 'gain_d_angular':
                self.gain_d_angular = p.value if p.value >= 0 else self.gain_d
            elif p.name == 'max_linear_vel':
                self.max_linear_vel = p.value
            elif p.name == 'max_angular_vel':
                self.max_angular_vel = p.value
            elif p.name == 'pos_threshold':
                self.pos_threshold = p.value
            elif p.name == 'rot_threshold':
                self.rot_threshold = p.value
        return SetParametersResult(successful=True)

    def _get_current_ee_pose(self):
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
        """cspace delta: position[0..6] = [dx, dy, dz, dqx, dqy, dqz, dqw]"""
        result = self._get_current_ee_pose()
        if result is None:
            return
        cur_pos, cur_quat = result

        dx, dy, dz = msg.position[0], msg.position[1], msg.position[2]
        dqx, dqy, dqz, dqw = msg.position[3], msg.position[4], msg.position[5], msg.position[6]

        new_pos = cur_pos + np.array([dx, dy, dz])

        dq_norm = math.sqrt(dqx**2 + dqy**2 + dqz**2 + dqw**2)
        if dq_norm < 1e-6:
            new_quat = cur_quat
        else:
            dqx, dqy, dqz, dqw = dqx/dq_norm, dqy/dq_norm, dqz/dq_norm, dqw/dq_norm
            cx, cy, cz, cw = cur_quat
            new_quat = [
                dqw*cx + dqx*cw + dqy*cz - dqz*cy,
                dqw*cy - dqx*cz + dqy*cw + dqz*cx,
                dqw*cz + dqx*cy - dqy*cx + dqz*cw,
                dqw*cw - dqx*cx - dqy*cy - dqz*cz,
            ]

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.base_link
        goal.pose.position.x = float(new_pos[0])
        goal.pose.position.y = float(new_pos[1])
        goal.pose.position.z = float(new_pos[2])
        goal.pose.orientation.x = float(new_quat[0])
        goal.pose.orientation.y = float(new_quat[1])
        goal.pose.orientation.z = float(new_quat[2])
        goal.pose.orientation.w = float(new_quat[3])

        self.goal_pose = goal
        self._prev_pos_error = np.zeros(3)
        self._prev_rot_err = np.zeros(3)

    def goal_pose_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        self._prev_pos_error = np.zeros(3)
        self._prev_rot_err = np.zeros(3)

    # ── Control loop ──────────────────────────────────────────────────

    def control_loop(self):
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

        pos_error = goal_pos - cur_pos
        rot_err_vec = quat_error_to_angular_vel(goal_quat, cur_quat)

        if (np.linalg.norm(pos_error) < self.pos_threshold and
                np.linalg.norm(rot_err_vec) < self.rot_threshold):
            self._prev_pos_error = np.zeros(3)
            self._prev_rot_err = np.zeros(3)
            self.twist_pub.publish(Twist())
            return

        dt = 1.0 / self.control_hz
        d_pos = (pos_error - self._prev_pos_error) / dt
        d_rot = (rot_err_vec - self._prev_rot_err) / dt
        self._prev_pos_error = pos_error.copy()
        self._prev_rot_err = rot_err_vec.copy()

        lin_vel_base = self.gain * pos_error + self.gain_d * d_pos
        ang_vel_base = self.gain_angular * rot_err_vec + self.gain_d_angular * d_rot

        qx, qy, qz, qw = cur_quat
        lin_vel_tool = rotate_vec_by_quat(lin_vel_base, -qx, -qy, -qz, qw)
        ang_vel_tool = rotate_vec_by_quat(ang_vel_base, -qx, -qy, -qz, qw)

        lin = lin_vel_tool.copy()
        ang = ang_vel_tool.copy()
        lin_norm = np.linalg.norm(lin)
        ang_norm = np.linalg.norm(ang)
        if lin_norm > self.max_linear_vel:
            lin *= self.max_linear_vel / lin_norm
        if ang_norm > self.max_angular_vel:
            ang *= self.max_angular_vel / ang_norm

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = float(lin[0]), float(lin[1]), float(lin[2])
        msg.angular.x, msg.angular.y, msg.angular.z = float(ang[0]), float(ang[1]), float(ang[2])
        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoJointBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
