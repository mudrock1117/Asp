#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from reportlab.platypus.figures import test1
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from codroid_msgs.msg import RobotInfo
from sensor_msgs.msg import JointState
import time
import copy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from math import pi

class CodroidMove(Node):
    def __init__(self):
        super().__init__('CodroidMove')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.robot_state = ""
        self.publisher_ = self.create_publisher(JointTrajectory, 'RobotMove', qos_profile)
        
        # 订阅RobotInfo主题以获取机器人状态
        self.robot_info_subscription = self.create_subscription(
            RobotInfo,
            'RobotInfo',
            self.robot_info_callback,
            10
        )

        # 订阅关节空间的目标位置主题以执行移动
        self.robot_goal_subscription = self.create_subscription(
            JointState,
            '/joint_goal_states',
            self.robot_goal_callback,
            10
        )

        self.robot_state = ""
        self.goal = None  # ✅ 初始化 goal 为 None

    def robot_goal_callback(self, msg):
        self.goal = msg
        self.test1()

    def test1(self):    
        msg1 = JointTrajectory()
        msg1.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        
        # 初始位置
        #p1 = JointTrajectoryPoint()
        #p1.positions = [-25.248/180.0*pi, 9.895/180.0*pi, 36.850/180.0*pi, -43.277/180.0*pi, 89.996/180.0*pi, -25.249/180.0*pi]
        #p1.time_from_start = Duration(sec=0, nanosec=500000000)
        #msg.points.append(p1)
        
        # 目标位置
        #p2 = JointTrajectoryPoint()
        #p2.positions = [-18.026/180.0*pi, 9.452/180.0*pi, 37.411/180.0*pi, -43.160/180.0*pi, 89.991/180.0*pi, -18.026/180.0*pi]
        #p2.time_from_start = Duration(sec=1, nanosec=0)
        #msg.points.append(p2)
        
        #p3 = JointTrajectoryPoint()
        #p3.positions = [-8.284/180.0*pi, 11.239/180.0*pi, 35.126/180.0*pi, -43.656/180.0*pi, 89.985/180.0*pi, -8.284/180.0*pi]
        #p3.time_from_start = Duration(sec=1, nanosec=500000000)
        #msg.points.append(p3)
        
        # 下降到目标位置
        #p4 = JointTrajectoryPoint()
        #p4.positions = [-2.173/180.0*pi, 13.826/180.0*pi, 31.663/180.0*pi, -44.534/180.0*pi, 89.98/180.0*pi, -2.173/180.0*pi]
        #p4.time_from_start = Duration(sec=2, nanosec=0)
        #msg.points.append(p4)
        
        # 回到预放置位置
        #p5 = JointTrajectoryPoint()
        #p5.positions = [6.662/180.0*pi, 20.566/180.0*pi, 21.688/180.0*pi, -47.773/180.0*pi, 89.975/180.0*pi, 6.662/180.0*pi]
        #p5.time_from_start = Duration(sec=2, nanosec=500000000)
        #msg.points.append(p5)
        
        # 回到初始位置
        #p6 = JointTrajectoryPoint()
        #p6.positions = [11.475/180.0*pi, 9.258/180.0*pi, 37.670/180.0*pi, -43.107/180.0*pi, 89.963/180.0*pi, 11.475/180.0*pi]
        #p6.time_from_start = Duration(sec=3, nanosec=0)
        #msg.points.append(p6)
        
        #p7 = JointTrajectoryPoint()
        #p7.positions = [17.395/180.0*pi, 7.124/180.0*pi, 40.273/180.0*pi, -42.635/180.0*pi, 89.957/180.0*pi, 17.395/180.0*pi]
        #p7.time_from_start = Duration(sec=3, nanosec=500000000)
        #msg.points.append(p7)
        
        #p8 = JointTrajectoryPoint()
        #p8.positions = [29.182/180.0*pi, -4.399/180.0*pi, 64.58/180.0*pi, -29.846/180.0*pi, 89.946/180.0*pi, 29.182/180.0*pi]
        #p8.time_from_start = Duration(sec=4, nanosec=0)
        #msg.points.append(p8)
        
        #p9 = JointTrajectoryPoint()
        #p9.positions = [0.0/180.0*pi, 0.0/180.0*pi, 0.0/180.0*pi, 0.0/180.0*pi, 0.0/180.0*pi, 0.0/180.0*pi]
        #p9.time_from_start = Duration(sec=4, nanosec=500000000)
        #msg.points.append(p9)
        
        p1 = JointTrajectoryPoint()
        # p1.positions = [-7.971189236824067e-06, 3.188475694729627e-06, 1.0447842009258266, -3.188475694729627e-05, 1.0447758020812938, -1.5942378473648134e-05]
        p1.positions = list(self.goal.position)
        p1.velocities = [1.3]
        msg1.points.append(p1)
        
        self.publisher_.publish(msg1)
        self.get_logger().info('Published trajectory message with %d points' % len(msg1.points))
        print(msg1)
        
    # def test2(self, robot_state):
    #     msg1 = JointTrajectory()
    #     msg1.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
    #     p10 = JointTrajectoryPoint()
    #     p10.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     p10.time_from_start = Duration(sec=1, nanosec=0)
    #     msg1.points.append(p10)
    #     while True:
    #         print(self.robot_state)
    #         if self.robot_state == "AutoReady":
    #             break
    #     self.publisher_.publish(msg1)
    #     self.get_logger().info('Published trajectory message with %d points' % len(msg1.points))
    #     print(msg1)
    
    # ---------- 轨迹 2：等状态 ----------
    # def wait_and_send(self):
    #     # 降低轮询频率，保护 CPU
    #     if self.robot_state != "AutoReady":
    #         #self.get_logger().info_throttle(5, 'Waiting for AutoReady ...')
    #         return
    #
    #     # 状态到位，停掉定时器，只发一次
    #     self.timer.cancel()
    #
    #     msg = JointTrajectory()
    #     msg.joint_names = [f"Joint{i}" for i in range(1, 7)]
    #     p = JointTrajectoryPoint()
    #     p.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     p.time_from_start = Duration(sec=7)
    #     msg.points.append(p)
    #
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Published zero-position trajectory.')

    # ---------- 回调 ----------
    def robot_info_callback(self, msg: RobotInfo):
        self.robot_state = msg.state   # 假定 msg.state 是 str
        
    #def robot_info_callback(self, msg):
    #    """接收并更新机器人状态信息"""
    #    self.robot_state = msg.state
        #if msg.state == "AutoReady":
            #self.Ready = "True"
        #else: 
            #self.Ready = "False"
        

# def main(args=None):
#     rclpy.init(args=args)
#     node = CodroidMoveTest()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

if __name__ == '__main__':
    # main()
    rclpy.init()
    node = CodroidMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
