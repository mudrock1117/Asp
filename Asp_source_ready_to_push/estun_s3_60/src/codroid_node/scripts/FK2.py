#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import sys, os, json, time
import numpy as np
from robotiq_interfaces.msg import GripperInput, GripperOutput
from codroid_msgs.msg import RobotInfo

# ===== 导入正运动学模块 =====
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = '/home/robot/estun_s3_60/src/codroid_node/scripts'
sys.path.insert(0, src_dir)
try:
    from estun_mdh_fk import frame1_to_base_coords, mdh_target_relative_to_tcp
except ImportError as e:
    print(f"导入错误: {e}")
    print(f"当前路径: {sys.path}")
    raise

# ===== YOLO 检测结果路径 =====
LATEST_DETECTIONS_PATH = "/home/robot/download/w/latest_detections.json"


def read_all_objects_m(path: str, max_age_s: float = 10.0):
    """读取 YOLO 检测结果 JSON（全部目标）"""
    try:
        if not os.path.exists(path):
            return []
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        ts = float(d.get("ts", 0.0))
        if (time.time() - ts) > max_age_s:
            return []
        detections = d.get("detections", [])
        pts = []
        for det in detections:
            cam = det.get("camera", {})
            x, y, z = cam.get("x_m"), cam.get("y_m"), cam.get("z_m")
            if None not in (x, y, z):
                pts.append((float(x), float(y), float(z)))
        return pts
    except Exception as e:
        print(f"读取检测 JSON 失败: {e}")
        return []


def point_on_line_from_object(p_obj_mm, p_tcp_mm, d_mm: float = 230.0):
    """从物体点出发，沿指向 TCP 的方向前进 d_mm（mm），返回新点（mm）"""
    p_obj = np.asarray(p_obj_mm, dtype=float).reshape(3)
    p_tcp = np.asarray(p_tcp_mm, dtype=float).reshape(3)
    v = p_tcp - p_obj
    L = np.linalg.norm(v)
    if L < 1e-9:
        return p_obj.copy()
    return p_obj + d_mm * (v / L)


class EstunRealtimeFK(Node):
    def __init__(self):
        super().__init__('estun_realtime_fk_test')

        self.base_offset_at_zero = [81.5, 77.0, -7.0]
        self.ext_len = 230.0
        self.d6 = 138.5
        self.base_sign = (1, 1, 1)
        self.rot90k = 0

        self.gripper = None
        self.robotinfo = False
        self.object = []
        self.grip = 0

        self.joint_names_expected = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]

        self.ignore_non_running = False
        self.ignore_fault = False

        # ---- ROS 接口 ----
        self.sub_robotinfo = self.create_subscription(
            RobotInfo, 'RobotInfo', self.robot_info_callback, 10
        )
        self.sub_gripper = self.create_subscription(
            GripperInput, 'gripper_input', self.gripper_callback, 10
        )
        self.pub_gripper = self.create_publisher(UInt8, 'gripper_goal', 10)
        self.goal_xyz_point = self.create_publisher(PointStamped, '/target_point', 10)
        self.sub_select = self.create_subscription(
            String, '/vision/target_select', self.cb_select, 10
        )

        self.q_rad = np.zeros(6, dtype=float)
        self.have_joints = False
        self.last_src = 'none'
        self.target_choice = 'p230'

        self.timer = self.create_timer(25, self.cb_timer)
        self.get_logger().info("【多目标抓取版】EstunRealtimeFK 启动完毕。")

    def robot_info_callback(self, msg):
        try:
            if self.ignore_non_running and msg.state != "AutoRunning":
                return
            if self.ignore_fault and int(msg.fault_flag) != 0:
                return

            jp = np.asarray(msg.joint_positions, dtype=float)
            if jp.size >= 6:
                self.q_rad = jp[:6]
                self.have_joints = True
                if self.last_src != 'robotinfo':
                    self.get_logger().info("关节源 = robotinfo/RobotInfo")
                    self.last_src = 'robotinfo'
        except Exception as e:
            self.get_logger().warn(f"RobotInfo 解析失败: {e}")

    def gripper_callback(self, msg2):
        self.gripper = msg2

    def cb_select(self, msg: String):
        v = (msg.data or "").strip().lower()
        if v in ('object', 'tcp', 'p230'):
            if v != self.target_choice:
                self.get_logger().info(f"切换目标选择: {self.target_choice} -> {v}")
            self.target_choice = v
        else:
            self.get_logger().warn(f"无效选择: {msg.data!r}，应为 object|tcp|p230")

    # ---------- 主循环 ----------
    def cb_timer(self):
        if not self.have_joints:
            return

        # 读取全部目标点
        objects = read_all_objects_m(LATEST_DETECTIONS_PATH, max_age_s=10.0)
        if not objects:
            self.get_logger().warn("未检测到可抓取目标。")
            return

        for (camera_x, camera_y, camera_z) in objects:
            self.get_logger().info(f"准备抓取目标: ({camera_x:.3f}, {camera_y:.3f}, {camera_z:.3f})")

            # 1) 相机坐标 -> 毫米
            p1_mm = np.array([camera_x, camera_y, camera_z]) * 1000.0

            # 2) 坐标变换
            try:
                p2_mm, T0E, _ = frame1_to_base_coords(
                    self.q_rad, p1_mm,
                    base_offset_at_zero=self.base_offset_at_zero,
                    d6=self.d6, base_sign=self.base_sign, rot90k=self.rot90k
                )
            except Exception as e:
                self.get_logger().error(f"FK 计算失败: {e}")
                continue

            # 3) 抓取准备
            p_tcp_mm = T0E[:3, 3].reshape(3)
            p230_mm = point_on_line_from_object(p2_mm, p_tcp_mm, d_mm=self.ext_len)
            object_m = np.asarray(p2_mm, dtype=float) / 1000.0

            grip = 62
            grip_msg = UInt8()
            grip_msg.data = grip
            self.pub_gripper.publish(grip_msg)

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base'
            msg.point.x = float(object_m[0]) - 0.023
            msg.point.y = float(object_m[1]) - 0.01
            if float(object_m[2]) + 0.2 <= 0.23:
                object_m[2] = 0.23 - 0.2
            msg.point.z = float(object_m[2]) + 0.2
            self.goal_xyz_point.publish(msg)
            time.sleep(4)

            grip = 228
            grip_msg.data = grip
            self.pub_gripper.publish(grip_msg)
            time.sleep(1.8)

            msg1 = PointStamped()
            msg1.header.stamp = self.get_clock().now().to_msg()
            msg1.header.frame_id = 'base'
            msg1.point.x = msg.point.x
            msg1.point.y = msg.point.y
            msg1.point.z = msg.point.z + 0.145
            self.goal_xyz_point.publish(msg1)
            print("UP")
            time.sleep(4)

            # 放置点
            msg3 = PointStamped()
            msg3.header.stamp = self.get_clock().now().to_msg()
            msg3.header.frame_id = 'base'
            msg3.point.x = float(0.0773)
            msg3.point.y = float(0.3363)
            msg3.point.z = float(0.3672)
            self.goal_xyz_point.publish(msg3)
            print("UP")
            time.sleep(5.5)

            grip = 0
            grip_msg.data = grip
            self.pub_gripper.publish(grip_msg)
            time.sleep(1.5)

            msg3 = PointStamped()
            msg3.header.stamp = self.get_clock().now().to_msg()
            msg3.header.frame_id = 'base'
            msg3.point.x = float(-0.3415)
            msg3.point.y = float(-0.0335)
            msg3.point.z = float(0.429)
            self.goal_xyz_point.publish(msg3)
            print("UP")
            time.sleep(4)

            self.get_logger().info("✅ 当前物体抓取完毕，继续下一个...\n")
            time.sleep(4)
        self.get_logger().info("🎯 所有检测到的物体已抓取完成。")


def main():
    rclpy.init()
    node = EstunRealtimeFK()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
