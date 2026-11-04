#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time


class IKTest(Node):
    def __init__(self):
        super().__init__('ik_test')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def test_ik(self):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK服务不可用')
            return False

        # 创建请求
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = 'estun'

        # 设置机器人状态
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = JointState()
        request.ik_request.robot_state.joint_state.header = Header()
        request.ik_request.robot_state.joint_state.header.frame_id = 'base_link'
        request.ik_request.robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        request.ik_request.robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 设置目标位姿
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header = Header()
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.pose = Pose()
        request.ik_request.pose_stamped.pose.position = Point(x=0.326764, y=0.109782, z=0.386671)
        request.ik_request.pose_stamped.pose.orientation = Quaternion(x=0.707, y=-0.707, z=0.0, w=0.0)

        self.get_logger().info('发送IK测试请求...')

        try:
            future = self.ik_client.call_async(request)
            start_time = time.time()

            while time.time() - start_time < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    response = future.result()
                    self.get_logger().info('IK测试成功!')
                    if hasattr(response, 'solution'):
                        self.get_logger().info(f'解算结果: {response.solution.joint_state.position}')
                    return True

            self.get_logger().error('IK测试超时')
            return False

        except Exception as e:
            self.get_logger().error(f'IK测试失败: {e}')
            return False


def main():
    rclpy.init()
    node = IKTest()
    success = node.test_ik()
    node.destroy_node()
    rclpy.shutdown()
    print(f"IK服务测试: {'成功' if success else '失败'}")


if __name__ == '__main__':
    main()