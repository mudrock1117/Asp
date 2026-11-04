#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, PositionIKRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math, asyncio, time


class SimpleMotionPlanner(Node):
    def __init__(self):
        super().__init__('simple_motion_planner')

        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待IK服务可用...')

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.goal_xyz_sub = self.create_subscription(PointStamped, '/target_point', self.target_point_callback, 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.publisher1 = self.create_publisher(JointState, '/joint_goal_states', qos_profile)

        self.current_joint_state = None
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.target_point = None
        self.label = False

        self.get_logger().info("等待 MoveGroup 服务器连接中...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("MoveGroup 已连接 ✅")

    def joint_state_callback(self, msg):
        if len(msg.name) >= 6:
            self.current_joint_state = msg

    def target_point_callback(self, msg1):
        self.target_point = msg1.point
        self.label = True
        self.get_logger().info(f"收到目标点: ({msg1.point.x:.3f}, {msg1.point.y:.3f}, {msg1.point.z:.3f})")

    async def move_to_pose(self, pose, joint_constraints=None):
        """使用逆运动学求解目标姿态"""
        for attempt in range(2):  # 最多两次尝试（一次正常，一次放宽）
            joint_target = await self._solve_ik(pose, joint_constraints)
            if joint_target:
                self.get_logger().info(f"IK 求解成功，目标关节角: {[round(j, 3) for j in joint_target]}")
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = self.joint_names
                js.position = joint_target
                self.publisher1.publish(js)
                return True
            else:
                self.get_logger().warn(f"第 {attempt+1} 次 IK 求解失败，尝试放宽约束重试...")
                if joint_constraints:
                    for jc in joint_constraints.values():
                        jc["tolerance_above"] *= 1.5
                        jc["tolerance_below"] *= 1.5
        self.get_logger().error("两次 IK 求解均失败 ❌")
        return False

    async def _solve_ik(self, target_pose, joint_constraints=None):
        """调用 MoveIt IK 服务"""
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = 'estun'
        request.ik_request.timeout.sec = 3
        request.ik_request.max_attempts = 5  # 使用 max_attempts 替代 attempts
        request.ik_request.avoid_collisions = False

        # 初始关节状态
        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = self.joint_names
        if self.current_joint_state and self.current_joint_state.position:
            state.joint_state.position = list(self.current_joint_state.position)
        else:
            state.joint_state.position = [0.0] * 6
        request.ik_request.robot_state = state

        # 目标位姿
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose = target_pose
        request.ik_request.pose_stamped = ps

        # 约束（若提供）
        if joint_constraints:
            constraints = Constraints()
            for jn, jc in joint_constraints.items():
                j = JointConstraint()
                j.joint_name = jn
                j.position = jc.get("position", 0.0)
                j.tolerance_above = jc.get("tolerance_above", 3.14)
                j.tolerance_below = jc.get("tolerance_below", 3.14)
                j.weight = jc.get("weight", 1.0)
                constraints.joint_constraints.append(j)
            request.ik_request.constraints = constraints

        future = self.ik_client.call_async(request)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().error(f"IK 调用失败: {e}")
                    return None

                code = response.error_code.val
                if code != 1:
                    self.get_logger().warn(f"IK 求解失败，错误码 {code}")
                    return None

                if not response.solution.joint_state.position:
                    self.get_logger().warn("IK 返回空解")
                    return None
                return list(response.solution.joint_state.position)
        return None

    def euler_to_quaternion_xyz(self, roll, pitch, yaw):
        """欧拉角 → 四元数"""
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        return [x / norm, y / norm, z / norm, w / norm]

    def create_pose(self, x, y, z, roll, pitch, yaw):
        """构造Pose消息"""
        qx, qy, qz, qw = self.euler_to_quaternion_xyz(roll, pitch, yaw)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        return pose


async def main():
    rclpy.init()
    node = SimpleMotionPlanner()

    node.get_logger().info("等待关节状态...")
    start = time.time()
    while node.current_joint_state is None and time.time() - start < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.current_joint_state:
        node.get_logger().error("未收到关节状态，退出")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        while rclpy.ok():
            while not node.label:
                rclpy.spin_once(node, timeout_sec=0.1)
                await asyncio.sleep(0.05)
            node.label = False

            tp = node.target_point
            pose = node.create_pose(tp.x, tp.y, tp.z, math.pi, 0.0, math.pi / 2)

            joint_constraints = {
                "joint2": {"position": -0.8, "tolerance_above": 2.0, "tolerance_below": 1.2},
                "joint3": {"position": 1.6, "tolerance_above": 2.0, "tolerance_below": 2.0},
                "joint5": {"position": 1.57, "tolerance_above": 1.8, "tolerance_below": 1.8}
            }

            success = await node.move_to_pose(pose, joint_constraints)
            node.get_logger().info(f"IK 执行: {'成功 ✅' if success else '失败 ❌'}")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
