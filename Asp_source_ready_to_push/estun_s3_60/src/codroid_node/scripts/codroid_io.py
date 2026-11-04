#!/usr/bin/env python3
import rclpy

from rclpy.action import ActionServer
import select
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration  # 添加这行导入
import socket
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from codroid_msgs.msg import RobotInfo

from sensor_msgs.msg import JointState  # 添加JointState消息导入

class CodroidIO(Node):
    def __init__(self):
        super().__init__('CodroidIO')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = ('192.168.101.100', 9005)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0', 9005))
        self.server_socket.setblocking(False)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info('Action服务器已启动')
        #self.server_socket.settimeout(0.5)
       

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.subscription = self.create_subscription(JointTrajectory, 'RobotMove', self.listener_callback, qos_profile)
        # 添加话题订阅验证
        self.get_logger().info(f"已订阅话题: {self.subscription.topic_name}")

        self.publisher = self.create_publisher(RobotInfo, 'RobotInfo', 10)
        print("Ready")
        
          # 添加JointState发布器##########################################################
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        ##################################################################################
        
        try:
            test_msg = json.dumps({"test": "connection"}).encode('utf-8')
            self.client_socket.sendto(test_msg, self.server_address)
            self.get_logger().info("测试消息已发送")
        except Exception as e:
            self.get_logger().error(f"发送测试消息失败: {e}")

        self.timer = self.create_timer(0.01, self.timer_callback)
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('接收到轨迹执行请求')
    
        # 获取目标轨迹
        trajectory = goal_handle.request.trajectory
    
        # 发布轨迹到RobotMove话题
        self.listener_callback(trajectory)  # 直接使用现有的listener_callback方法
    
        # 这里应该添加实际执行监控逻辑
        # 暂时简单等待并返回成功
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def timer_callback(self):
        try:
            data, _ = self.server_socket.recvfrom(1024)  # 接收服务器响应
            json_data = json.loads(data.decode('utf-8'))
            # 打印接收到的原始JSON数据（用于调试）
            #self.get_logger().info(f"Received from {addr}: {json_data}")
            robot_info = RobotInfo()
            robot_info.joint_positions = json_data["joint_positions"]
            robot_info.end_positions = json_data["end_positions"]
            robot_info.state = json_data["state"]
            robot_info.fault_flag = json_data["fault_flag"]
            self.publisher.publish(robot_info)
            #print(robot_info)
            
            # 发布JointState消息##########################################
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]  # 根据实际关节名调整
            joint_state.position = json_data["joint_positions"]
            # 如果没有速度信息，可以留空或设置为零
            joint_state.velocity = [0.0] * len(joint_state.position) if "joint_velocities" not in json_data else json_data["joint_velocities"]
            self.joint_state_publisher.publish(joint_state)
            ###############################################################
            
            #print(robot_info)
        #except:
        except BlockingIOError:
            # 没有数据可读，这是正常的非阻塞行为
            return
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析错误: {e}")
            pass
        except KeyError as e:
            self.get_logger().error(f"JSON数据中缺少键: {e}")
            pass
        return

    def listener_callback(self, msg):
        self.get_logger().info("listener_callback 被调用")
        #json_str = self.to_json(msg)
        #print(json_str)
        #self.client_socket.sendto(json_str.encode('utf-8'), self.server_address)
        try:
            #print("json_str")
            json_str = self.to_json(msg)
            #print(json_str)
            self.client_socket.sendto(json_str.encode('utf-8'), self.server_address)
            #print(json_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"处理RobotMove消息失败: {e}")

    def to_json(self, msg):
        data = {
            'joint_names': [joint_name for joint_name in msg.joint_names],
            'points': [{'positions': [_ for _ in point.positions],
                        'velocities': [_ for _ in point.velocities],
                        'accelerations': [_ for _ in point.accelerations]} for point in msg.points]
        }
        return json.dumps(data, default=str)
    
if __name__ == '__main__':
    rclpy.init()
    node = CodroidIO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
