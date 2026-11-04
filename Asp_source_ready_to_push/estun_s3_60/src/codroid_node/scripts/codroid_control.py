#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from math import pi

class CodroidArmController(Node):
    def __init__(self):
        super().__init__('codroid_arm_controller')
        self.publisher = self.create_publisher(JointTrajectory, 'RobotMove', 10)
        
    def move_to_joint_positions(self, positions, time_from_start=5.0):
        """移动到指定关节角度"""
        msg = JointTrajectory()
        msg.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rclpy.duration.Duration(seconds=time_from_start).to_msg()
        
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f'Moving to position: {positions}')
    
    def generate_trajectory(self, start_pos, end_pos, num_points=10, duration=5.0):
        """生成平滑轨迹"""
        msg = JointTrajectory()
        msg.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        
        # 线性插值生成轨迹点
        for i in range(num_points):
            ratio = i / (num_points - 1)
            point = JointTrajectoryPoint()
            point.positions = [
                start_pos[j] + ratio * (end_pos[j] - start_pos[j]) for j in range(6)
            ]
            point.time_from_start = rclpy.duration.Duration(
                seconds=ratio * duration
            ).to_msg()
            msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Executing trajectory from {start_pos} to {end_pos}')

def main(args=None):
    rclpy.init(args=args)
    controller = CodroidArmController()
    
    # 示例：移动到初始位置
    controller.move_to_joint_positions([0.8, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # 保持节点运行
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
