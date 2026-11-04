#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, RobotState, \
    PositionIKRequest
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
from math import pi
import asyncio


class SimpleMotionPlanner(Node):
    def __init__(self):
        super().__init__('simple_motion_planner')

        # Action客户端 - 用于与MoveGroup通信
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # 创建IK服务客户端 - 统一使用 self.ik_client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')  # <- 修复1: 统一变量名
        while not self.ik_client.wait_for_service(timeout_sec=1.0):  # <- 修复2
            self.get_logger().info('服务不可用，等待中...')

        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 订阅目标位置
        self.goal_xyz_sub = self.create_subscription(
            PointStamped,  # 使用PointStamped，包含坐标和坐标系信息
            '/target_point',  # 话题名称，可根据需要修改
            self.target_point_callback,
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # 发布关节空间目标状态
        self.publisher1 = self.create_publisher(JointState, '/joint_goal_states', qos_profile)

        self.label = False
        self.current_joint_state = None
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        # 目标点相关变量
        self.target_point = None
        self.target_frame = "base_link"  # 默认坐标系

        # 等待MoveGroup Action服务器
        self.get_logger().info("等待MoveGroup Action服务器...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("MoveGroup Action服务器已连接")

        # 等待IK服务 - 添加超时机制
        self.get_logger().info("等待IK服务...")
        if not self.ik_client.wait_for_service(timeout_sec=10.0):  # <- 这里不再报错，因为已统一变量名
            self.get_logger().error("IK服务不可用，超时")
        # 可以在这里决定是否继续运行
        else:
            self.get_logger().info("IK服务已连接")

    def joint_state_callback(self, msg):
        """保存当前关节状态"""
        # 确保关节状态消息包含所有关节
        if len(msg.name) == len(self.joint_names):
            self.current_joint_state = msg

    def target_point_callback(self, msg1):
        """接收目标坐标"""
        try:
            self.target_point = msg1.point
            self.label = True
            print(msg1.point)
        except Exception as e:
            self.get_logger().error(f"处理目标坐标时发生错误: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    async def move_to_joints(self, joint_positions):
        """移动到指定关节角度"""
        return await self._send_joint_move_request(joint_positions=joint_positions)

    async def move_to_pose(self, pose, joint_constraints=None):
        """移动到指定位姿（先使用逆运动学求解，再使用关节空间规划）"""
        # 使用逆运动学将笛卡尔位姿转换为关节角度
        joint_target = await self._solve_ik(pose, joint_constraints)

        # 修复4: 移除错误的代码，简化逻辑
        if joint_target is not None:
            self.get_logger().info(f"逆运动学求解结果: {[f'{pos:.3f}' for pos in joint_target]}")
            # if joint_target[0] - self.current_joint_state.position[0] > 1.57:
            #     return await self._send_joint_move_request(joint_positions=joint_target)
            # else:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = joint_target
            self.publisher1.publish(joint_state_msg)
            return False

        else:
            self.get_logger().error("逆运动学求解失败")
            return False

    async def _send_joint_move_request(self, joint_positions):
        """发送关节空间移动请求到MoveGroup"""
        # 检查是否有当前关节状态
        if self.current_joint_state is None:
            self.get_logger().error("没有当前关节状态，无法规划")
            return False

        # 创建MoveGroup目标
        goal_msg = MoveGroup.Goal()

        # 设置规划参数
        goal_msg.request.group_name = "estun"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        goal_msg.request.pipeline_id = "ompl"

        # 设置起始状态
        goal_msg.request.start_state.joint_state = self.current_joint_state
        goal_msg.request.start_state.joint_state.header.frame_id = "base_link"

        # 设置工作空间参数
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        # 设置目标约束 - 使用关节空间目标
        goal_constraints = Constraints()

        # 关节空间目标
        for i, joint_name in enumerate(self.joint_names):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = joint_positions[i]
            jc.tolerance_above = 0.0001
            jc.tolerance_below = 0.0001
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        # 添加目标约束到请求
        goal_msg.request.goal_constraints.append(goal_constraints)

        # 设置规划选项
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.get_logger().info("发送关节空间移动请求...")

        try:
            # 发送目标并等待结果
            future = self.move_group_client.send_goal_async(goal_msg)
            # response = await future
            # print("ready")
            #
            # if response.accepted:
            #     self.get_logger().info("移动请求已被接受")
            #     result_future = response.get_result_async()
            #     result = await result_future
            #
            #     if result.status == 4:  # SUCCEEDED
            #         self.get_logger().info("移动执行成功!")
            #         return True
            #     else:
            #         self.get_logger().error(f"移动执行失败，状态码: {result.status}")
            #         return False
            # else:
            #     self.get_logger().error("移动请求被拒绝")
            #     return False

        except Exception as e:
            self.get_logger().error(f"执行移动时发生错误: {str(e)}")
            return False

    async def _solve_ik(self, target_pose, joint_constraints=None):
        """使用IK服务进行逆运动学求解 - 按照ik.py的方式"""
        # 创建IK请求
        request = GetPositionIK.Request()

        # 构建IK请求内容 - 完全按照ik.py的方式
        request.ik_request = PositionIKRequest()

        # 设置组名
        request.ik_request.group_name = 'estun'
        request.ik_request.timeout.sec = 5

        # 构建机器人状态
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = JointState()

        # 设置关节状态头信息
        request.ik_request.robot_state.joint_state.header = Header()
        request.ik_request.robot_state.joint_state.header.stamp.sec = 0
        request.ik_request.robot_state.joint_state.header.stamp.nanosec = 0
        request.ik_request.robot_state.joint_state.header.frame_id = 'base_link'

        # 设置关节名称和位置 - 使用全零状态
        request.ik_request.robot_state.joint_state.name = self.joint_names
        # request.ik_request.robot_state.joint_state.position = list(self.current_joint_state.position)
        # 修复：检查当前关节状态是否存在
        if self.current_joint_state is not None and hasattr(self.current_joint_state, 'position'):
            request.ik_request.robot_state.joint_state.position = list(self.current_joint_state.position)
            self.get_logger().info("使用当前关节状态作为初始状态")
        else:
            # 如果没有当前关节状态，使用全零状态
            request.ik_request.robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("使用全零关节状态作为初始状态")

        # 构建目标位姿
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header = Header()
        request.ik_request.pose_stamped.header.stamp.sec = 0
        request.ik_request.pose_stamped.header.stamp.nanosec = 0
        request.ik_request.pose_stamped.header.frame_id = 'base_link'

        # 设置位置
        request.ik_request.pose_stamped.pose.position = Point()
        request.ik_request.pose_stamped.pose.position.x = target_pose.position.x
        request.ik_request.pose_stamped.pose.position.y = target_pose.position.y
        request.ik_request.pose_stamped.pose.position.z = target_pose.position.z

        # 设置方向
        request.ik_request.pose_stamped.pose.orientation = Quaternion()
        request.ik_request.pose_stamped.pose.orientation.x = target_pose.orientation.x
        request.ik_request.pose_stamped.pose.orientation.y = target_pose.orientation.y
        request.ik_request.pose_stamped.pose.orientation.z = target_pose.orientation.z
        request.ik_request.pose_stamped.pose.orientation.w = target_pose.orientation.w

        # 添加详细的调试信息
        self.get_logger().info("使用全零关节状态作为初始状态")
        self.get_logger().info(
            f"目标位姿 - 位置: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        self.get_logger().info(
            f"目标位姿 - 姿态: ({target_pose.orientation.x:.3f}, {target_pose.orientation.y:.3f}, {target_pose.orientation.z:.3f}, {target_pose.orientation.w:.3f})")
        self.get_logger().info("发送IK求解请求...")

        # 添加关节约束
        if joint_constraints:
            constraints = Constraints()

            for joint_name, constraint_params in joint_constraints.items():
                if joint_name in self.joint_names:
                    jc = JointConstraint()
                    jc.joint_name = joint_name

                    # 设置约束参数
                    if 'position' in constraint_params:
                        jc.position = constraint_params['position']
                    if 'tolerance_above' in constraint_params:
                        jc.tolerance_above = constraint_params['tolerance_above']
                    if 'tolerance_below' in constraint_params:
                        jc.tolerance_below = constraint_params['tolerance_below']
                    if 'weight' in constraint_params:
                        jc.weight = constraint_params['weight']
                    else:
                        jc.weight = 1.0

                    constraints.joint_constraints.append(jc)

            request.ik_request.constraints = constraints
            self.get_logger().info(f"添加了 {len(constraints.joint_constraints)} 个关节约束")

        # 调用IK服务
        future = self.ik_client.call_async(request)
        print(future)  # 打印Future对象，与ik.py保持一致

        # 等待响应
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(f'IK服务调用失败: {e}')
                    return None
                else:
                    self.get_logger().info('IK解算成功!')
                    # 处理响应，例如打印关节位置
                    if response.solution.joint_state.position:
                        self.get_logger().info(
                            f'解算结果: {response.solution.joint_state.position}')
                        return response.solution.joint_state.position
                break

        return response.solution.joint_state.position

    def _get_ik_error_message(self, error_code):
        """根据IK错误码返回描述信息"""
        error_messages = {
            1: "成功",
            -1: "失败",
            -2: "超时",
            -3: "超出限制",
            -4: "无法求解",
            -5: "起始状态无效",
            -6: "目标不可达",
            -7: "避障约束",
            -12: "其他错误"
        }
        return error_messages.get(error_code, f"未知错误码: {error_code}")

    def euler_to_quaternion_xyz(self, roll, pitch, yaw):
        """将欧拉角转换为四元数 (XYZ顺序)"""
        # XYZ顺序：先绕X轴旋转(roll)，再绕Y轴旋转(pitch)，最后绕Z轴旋转(yaw)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

    def create_pose(self, x, y, z, roll, pitch, yaw):
        """使用欧拉角创建位姿消息，内部转换为四元数供MoveIt2使用（使用XYZ顺序）"""
        # 使用XYZ顺序的欧拉角到四元数转换
        qx, qy, qz, qw = self.euler_to_quaternion_xyz(roll, pitch, yaw)

        # 创建Pose消息
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # 验证四元数
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if abs(norm - 1.0) > 0.001:
            self.get_logger().warning(f"四元数不是单位四元数，模长: {norm}")

        return pose


async def main():
    rclpy.init()
    node = SimpleMotionPlanner()

    # 等待关节状态信息
    node.get_logger().info("等待关节状态信息...")
    import time
    start_time = time.time()
    while node.current_joint_state is None and (time.time() - start_time) < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_joint_state is None:
        node.get_logger().error("在超时时间内未收到关节状态信息")
        node.destroy_node()
        rclpy.shutdown()
        return

    # node.get_logger().info("节点已就绪，输入命令:")
    # node.get_logger().info("1 - 回零 (关节空间)")
    # node.get_logger().info("2 - 测试位置 (关节空间)")
    # node.get_logger().info("3 - 测试位姿 (先逆运动学求解，再关节空间规划)")
    # node.get_logger().info("4 - 测试简单位姿 (与ik.py相同的位姿)")
    # node.get_logger().info("q - 退出")

    try:
        while rclpy.ok():
            # 简单的命令行交互
            # user_input = input("请输入命令: ").strip()
            while not node.label:
                rclpy.spin_once(node, timeout_sec=0.1)  # 添加这行处理ROS消息
                await asyncio.sleep(0.1)  # 使用异步睡眠而不是time.sleep()
            # if user_input == '1':
            #     # 回零位置
            #     home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #     success = await node._send_joint_move_request(home_position)
            #     node.get_logger().info(f"回零: {'成功' if success else '失败'}")
            #
            # elif user_input == '2':
            #     # 测试关节位置
            #     test_position = [0.0, -0.0193, 1.5534, 0.0, 1.5618, 0.0]
            #     success = await node._send_joint_move_request(test_position)
            #     node.get_logger().info(f"测试关节位置: {'成功' if success else '失败'}")

            # elif user_input == '3':
            # 测试复杂笛卡尔位姿
            if node.label:
                node.label = False
            print(node.target_point)
            test_pose = node.create_pose(node.target_point.x, node.target_point.y, node.target_point.z, 179.8 / 180.0 * pi, 0.0 / 180.0 * pi,
                                         90.2 / 180.0 * pi)
            # test_pose = node.create_pose(0.326764, 0.109782, 0.286671, 180.0 / 180.0 * pi, -0.0 / 180.0 * pi,
            #                                  -90.0 / 180.0 * pi)
            # 使用您提供的精确约束条件
            joint_constraints = {
                "joint2": {
                    "position": -0.8,
                    "tolerance_above": 1.8,
                    "tolerance_below": 1.1
                },
                "joint3": {
                    "position": 1.6,
                    "tolerance_above": 1.8,
                    "tolerance_below": 1.8
                },
                "joint4": {
                    "position": 0.0,
                    "tolerance_above": 1.2,
                    "tolerance_below": 1.58
                },
                "joint5": {
                    "position": 1.57,
                    "tolerance_above": 1.57,
                    "tolerance_below": 1.58
                }
            }
            success = await node.move_to_pose(test_pose, joint_constraints)
            node.get_logger().info(f"测试复杂笛卡尔位姿: {'成功' if success else '失败'}")

            # elif user_input == 'q':
            #     break
            # else:
            #     node.get_logger().info("未知命令")

            # 处理ROS事件
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
