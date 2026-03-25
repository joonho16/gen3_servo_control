#!/usr/bin/env python3
"""
MoveIt planning scene manager for Gen3.
장애물(box/sphere/cylinder)을 planning scene에 추가/제거합니다.

Usage:
  # 장애물 추가
  ros2 run gen3_servo_control scene_manager add box wall1 0.6 0.0 0.5 -- 0.02 0.4 0.6
  ros2 run gen3_servo_control scene_manager add sphere ball1 0.4 0.2 0.3 -- 0.1
  ros2 run gen3_servo_control scene_manager add cylinder cyl1 0.3 -0.2 0.4 -- 0.05 0.3

  # 장애물 제거
  ros2 run gen3_servo_control scene_manager remove wall1

  # 전체 제거
  ros2 run gen3_servo_control scene_manager clear
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
import time


class SceneManager(Node):
    def __init__(self):
        super().__init__('scene_manager')
        self.pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.scene_pub = self.create_publisher(PlanningScene, '/monitored_planning_scene', 10)
        # Publisher가 연결될 때까지 대기
        time.sleep(0.5)

    def add_box(self, name, x, y, z, sx, sy, sz, frame='base_link'):
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = name
        obj.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [sx, sy, sz]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        self.pub.publish(obj)
        self.get_logger().info(f'Added box [{name}] at ({x}, {y}, {z}), size ({sx}, {sy}, {sz})')

    def add_sphere(self, name, x, y, z, radius, frame='base_link'):
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = name
        obj.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [radius]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        self.pub.publish(obj)
        self.get_logger().info(f'Added sphere [{name}] at ({x}, {y}, {z}), radius {radius}')

    def add_cylinder(self, name, x, y, z, radius, height, frame='base_link'):
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = name
        obj.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [height, radius]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        self.pub.publish(obj)
        self.get_logger().info(f'Added cylinder [{name}] at ({x}, {y}, {z}), r={radius}, h={height}')

    def remove(self, name, frame='base_link'):
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        self.pub.publish(obj)
        self.get_logger().info(f'Removed [{name}]')

    def clear(self, frame='base_link'):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.clear()
        # REMOVE ALL via empty PlanningScene diff
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = 'all'
        obj.operation = CollisionObject.REMOVE
        self.pub.publish(obj)
        self.get_logger().info('Cleared all collision objects')


def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv[1:]

    if not argv:
        print(__doc__)
        return

    node = SceneManager()

    cmd = argv[0]

    try:
        if cmd == 'add':
            shape = argv[1]   # box / sphere / cylinder
            name = argv[2]
            x, y, z = float(argv[3]), float(argv[4]), float(argv[5])

            if shape == 'box':
                sx, sy, sz = float(argv[6]), float(argv[7]), float(argv[8])
                node.add_box(name, x, y, z, sx, sy, sz)
            elif shape == 'sphere':
                node.add_sphere(name, x, y, z, float(argv[6]))
            elif shape == 'cylinder':
                node.add_cylinder(name, x, y, z, float(argv[6]), float(argv[7]))
            else:
                print(f'Unknown shape: {shape}. Use box / sphere / cylinder')

        elif cmd == 'remove':
            node.remove(argv[1])

        elif cmd == 'clear':
            node.clear()

        else:
            print(f'Unknown command: {cmd}. Use add / remove / clear')

    except (IndexError, ValueError) as e:
        print(f'Error: {e}')
        print(__doc__)

    time.sleep(0.3)  # publish 전달 대기
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
