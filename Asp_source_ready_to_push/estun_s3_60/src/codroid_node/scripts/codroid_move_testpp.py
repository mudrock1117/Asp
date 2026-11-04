#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration  # 添加这行导入
from moveit_msgs.msg import DisplayTrajectory  # 添加这行导入
from codroid_msgs.msg import RobotInfo  # 导入RobotInfo消息类型
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from math import pi
import threading
import queue

class CodroidMoveTest(Node):
    def __init__(self):
        super().__init__('CodroidMoveTest')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.publisher = self.create_publisher(JointTrajectory, 'RobotMove', qos_profile)
        
        # 机械臂状态变量、
        self.robot_state = ""
        
        # 订阅/display_planned_path主题
        self.subscription = self.create_subscription(
            DisplayTrajectory, 
            '/display_planned_path', 
            self.display_planned_path_callback, 
            qos_profile
        )
        
        # 订阅RobotInfo主题以获取机器人状态
        self.robot_info_subscription = self.create_subscription(
            RobotInfo,
            'RobotInfo',
            self.robot_info_callback,
            10
        )

        # 线程安全队列
        self.point_queue = queue.Queue()
        
        time.sleep(0.2)
        
    def format_float_array(self, arr, precision=15):
        """将浮点数数组中的每个元素格式化为指定精度，取消科学计数法"""
        if arr is None or len(arr) == 0:
            return arr
        return [float(f"{x:.{precision}f}") for x in arr]
        
    def robot_info_callback(self, msg):
        """接收并更新机器人状态信息"""
        self.robot_state = msg.state
        
    def display_planned_path_callback(self, msg):
        self.get_logger().info('接收到MoveIt2规划的轨迹: "msg"')
        
        # 检查是否有轨迹数据
        if not msg.trajectory:
            self.get_logger().warning('DisplayTrajectory消息中没有轨迹数据')
            return
            
        # 提取第一条轨迹（通常只有一条）
        trajectory = msg.trajectory[0]
        
        # 检查是否有关节轨迹
        if not hasattr(trajectory, 'joint_trajectory') or not trajectory.joint_trajectory.points:
            self.get_logger().warning('轨迹中没有关节轨迹数据')
            return 
            
        joint_traj = trajectory.joint_trajectory
        self.InQueue(joint_traj)


    def InQueue(self, joint_trajectory):
        # 1. 保存关节名（后面重建轨迹时直接赋值）
        self.joint_names = joint_trajectory.joint_names

        # 2. 逐点入队
        for pt in joint_trajectory.points:
            self.point_queue.put(pt)          # 线程安全
            
        self.get_logger().info(f'已入队 {len(joint_trajectory.points)} 个点')
        # 3.分组出队
        while not self.point_queue.empty():
            self.OutQueue()


    def OutQueue(self):
        # 查看全局变量，机械臂是否处在待命状态
        global Ready
        
        
        Traj_OutQueue = JointTrajectory()
        Traj_OutQueue.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        #Traj_OutQueue.header.stamp = self.get_clock().now().to_msg()
        #dt = 0.01                         # 10 ms 一个点
        #t  = 0.0                          # 累计时间
        
        # 一条指令中含有最多100个轨迹点
        number = 100
        while not self.point_queue.empty() and number > 0:
            Traj = self.point_queue.get()

            # 格式化positions
            if Traj.positions:
                Traj.positions = self.format_float_array(Traj.positions)
            
            # 格式化velocities
            #if Traj.velocities:
            #    Traj.velocities = self.format_float_array([0.5])
            Traj.velocities = []
            
            # 格式化accelerations
            #if Traj.accelerations:
            #    Traj.accelerations = self.format_float_array(Traj.accelerations)
            Traj.accelerations = []
                
            # ---------- 关键：写时间 ----------
            #sec = int(t)
            #nanosec = int((t - sec) * 1e9)
            #Traj.time_from_start = Duration(sec=sec, nanosec=nanosec)
            # ------------------------------------
            Traj_OutQueue.points.append(Traj)
            #t += dt
            number = number - 1
        print("Traj_OutQueue")
        print(Traj_OutQueue)   
        
        # 等待机械臂空闲
        while self.robot_state != "AutoReady":
            time.sleep(0.1)
        
        # 将轨迹点装入相关话题中
        self.publisher.publish(Traj_OutQueue)
        self.get_logger().info('Published trajectory message with %d points' % len(Traj_OutQueue.points))
        



if __name__ == '__main__':
    rclpy.init()
    node = CodroidMoveTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
