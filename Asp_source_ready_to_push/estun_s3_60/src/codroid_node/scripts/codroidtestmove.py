#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import DisplayTrajectory
from codroid_msgs.msg import RobotInfo
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, RobotState
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
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
        
        self.robot_state = ""
        self.current_joint_state = None

        # 订阅规划结果
        self.subscription = self.create_subscription(
            DisplayTrajectory, 
            '/display_planned_path', 
            self.display_planned_path_callback, 
            qos_profile
        )
        
        # 订阅机器人状态
        self.robot_info_subscription = self.create_subscription(
            RobotInfo,
            'RobotInfo',
            self.robot_info_callback,
            10
        )

        # 订阅关节状态（关键！）
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 订阅目标位姿
        self.target_pose_subscription = self.create_subscription(
            Pose,
            '/target_pose',
            self.target_pose_callback,
            10
        )
        self.get_logger().info("✅ 已订阅 /target_pose")

        # MoveIt Action 客户端
        self.move_group_action_client = ActionClient(self, MoveGroup, '/move_action')
        while not self.move_group_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待 MoveIt Action Server...')
        self.get_logger().info("✅ MoveIt Action 客户端已就绪")

        self.point_queue = queue.Queue()
        time.sleep(0.2)

    def joint_state_callback(self, msg):
        """保存最新的关节状态"""
        self.current_joint_state = msg
        # self.get_logger().info(f"🔄 更新关节状态: {msg.position}")

    def get_safe_start_state(self):
        """安全姿态（仅当获取不到真实状态时使用）"""
        safe_state = JointState()
        safe_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        safe_state.position = [0.0, -1.0, 1.5, 0.0, 0.0, 0.0]  # 示例安全姿态
        return safe_state

    def format_float_array(self, arr, precision=11):
        if arr is None or len(arr) == 0:
            return arr
        return [float(f"{x:.{precision}f}") for x in arr]

    def robot_info_callback(self, msg):
        self.robot_state = msg.state

    def display_planned_path_callback(self, msg):
        self.get_logger().info('接收到MoveIt2规划的轨迹')
        if not msg.trajectory:
            self.get_logger().warning('没有轨迹数据')
            return
        trajectory = msg.trajectory[0]
        if not hasattr(trajectory, 'joint_trajectory') or not trajectory.joint_trajectory.points:
            self.get_logger().warning('没有关节轨迹数据')
            return 
        self.InQueue(trajectory.joint_trajectory)

    def target_pose_callback(self, pose: Pose):
        self.get_logger().info(f"🎯 收到程序化目标位姿: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
        
        # 🚀 关键：等待真实当前关节状态
        start_joint_state = None
        timeout = 5.0
        start_time = time.time()
        
        while start_joint_state is None and (time.time() - start_time) < timeout:
            if self.current_joint_state is not None:
                start_joint_state = self.current_joint_state
                self.get_logger().info("✅ 获取到当前真实关节状态")
            else:
                self.get_logger().warn("⏳ 等待 /joint_states 数据...")
                time.sleep(0.1)
        
        if start_joint_state is None:
            self.get_logger().error("❌ 未获取到关节状态，使用安全姿态")
            start_joint_state = self.get_safe_start_state()

        request = MotionPlanRequest()
        request.group_name = "estun"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        # 设置起始状态 = 当前真实状态
        robot_state = RobotState()
        robot_state.joint_state = start_joint_state
        request.start_state = robot_state

        # 设置目标约束
        constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "link6"
        pos_constraint.constraint_region.primitive_poses.append(pose)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 0.1, 0.1]
        pos_constraint.constraint_region.primitives.append(box)
        constraints.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "link6"
        ori_constraint.orientation = pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 1.0
        ori_constraint.absolute_y_axis_tolerance = 1.0
        ori_constraint.absolute_z_axis_tolerance = 1.0
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)

        request.goal_constraints.append(constraints)

        # 设置 plan_only
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = True

        self.get_logger().info("正在请求 MoveIt 规划...")
        future = self.move_group_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ MoveIt 目标被拒绝')
            return
        self.get_logger().info('✅ MoveIt 目标被接受，等待规划结果...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('🎉 MoveIt 规划成功！')
        else:
            self.get_logger().error(f'❌ MoveIt 规划失败，错误码: {result.error_code.val}')

    def InQueue(self, joint_trajectory):
        self.joint_names = joint_trajectory.joint_names
        for pt in joint_trajectory.points:
            self.point_queue.put(pt)
        self.get_logger().info(f'已入队 {len(joint_trajectory.points)} 个点')
        while not self.point_queue.empty():
            self.OutQueue()

    def OutQueue(self):
        Traj_OutQueue = JointTrajectory()
        # 🚀 强制使用大写关节名，适配你的机械臂
        Traj_OutQueue.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        
        dt = 0.01
        t = 0.0
        number = 100

        while not self.point_queue.empty() and number > 0:
            Traj = self.point_queue.get()

            if Traj.positions:
                Traj.positions = self.format_float_array(Traj.positions)
            
            if Traj.velocities:
                if len(Traj.velocities) != len(Traj_OutQueue.joint_names):
                    Traj.velocities = [1.5] * len(Traj_OutQueue.joint_names)
                else:
                    Traj.velocities = self.format_float_array(Traj.velocities)
            
            if Traj.accelerations:
                Traj.accelerations = self.format_float_array(Traj.accelerations)

            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            Traj.time_from_start = Duration(sec=sec, nanosec=nanosec)
            Traj_OutQueue.points.append(Traj)
            t += dt
            number -= 1

        # 等待机械臂空闲
        while self.robot_state != "AutoReady":
            time.sleep(0.1)

        self.publisher.publish(Traj_OutQueue)
        self.get_logger().info(f'✅ 已发布轨迹，包含 {len(Traj_OutQueue.points)} 个点')

def main(args=None):
    rclpy.init(args=args)
    node = CodroidMoveTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
