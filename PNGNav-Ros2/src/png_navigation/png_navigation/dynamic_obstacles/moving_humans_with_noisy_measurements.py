#!/usr/bin/python3.8
import time

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Pose, PoseArray


class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__('pose_array_publisher')

        # 创建两个发布者
        self.pose_array_pub = self.create_publisher(PoseArray, 'dr_spaam_detections', 10)  # 发布模拟检测结果，包括带有高斯噪声的位姿，模拟了探测过程中的误差。
        self.real_pose_array_pub = self.create_publisher(PoseArray, 'gt_human_positions', 10)  # 发布“真实”位置，代表目标的实际位置，便于对比分析
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_pose_array)

        self.px = 1.0
        self.py = -0.8
        self.count_turn = 40
        self.vx = -1 / 20.0
        self.vy = 0.0
        self.count = 0
        self.ts = time.time()

    def publish_pose_array(self):
        pose_array_msg = PoseArray()
        real_pose_array_msg = PoseArray()

        if self.count > self.count_turn:
            self.vx = -self.vx
            self.count = 0

        self.px += self.vx
        self.py += self.vy
        self.count += 1

        for i in range(1):
            pose = Pose()
            pose.position.x = self.px + np.random.randn() * 0.01
            pose.position.y = self.py + np.random.randn() * 0.01
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            pose_array_msg.poses.append(pose)

        if time.time() - self.ts > 2:
            pose = Pose()
            pose.position.x = -1.5 + np.random.randn() * 0.01
            pose.position.y = 0.5 + np.random.randn() * 0.01
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            pose_array_msg.poses.append(pose)

        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "map"

        pose = Pose()
        pose.position.x = self.px
        pose.position.y = self.py
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        real_pose_array_msg.poses.append(pose)

        if time.time() - self.ts > 2:
            pose = Pose()
            pose.position.x = -1.5
            pose.position.y = 0.5
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            real_pose_array_msg.poses.append(pose)

        real_pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        real_pose_array_msg.header.frame_id = "map"

        # 发布消息
        self.pose_array_pub.publish(pose_array_msg)
        self.real_pose_array_pub.publish(real_pose_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()