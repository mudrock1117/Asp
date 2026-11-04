#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import sys  # ← 添加这一行
import os, json, time
import numpy as np
from robotiq_interfaces.msg import GripperInput, GripperOutput
from codroid_msgs.msg import RobotInfo   # ←★改这里
from std_msgs.msg import UInt8  # 添加这行导入
# 获取当前脚本所在目录并添加到 Python 路径
# current_dir = os.path.dirname(os.path.abspath(__file__))
# sys.path.insert(0, current_dir)

# 获取当前脚本所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 正确设置源码目录路径
src_dir = '/home/robot/estun_s3_60/src/codroid_node/scripts'  # 直接使用绝对路径
sys.path.insert(0, src_dir)

# 现在可以导入同目录的模块
try:
    from estun_mdh_fk import frame1_to_base_coords, mdh_target_relative_to_tcp
except ImportError as e:
    print(f"导入错误: {e}")
    print(f"当前路径: {sys.path}")
    raise

# ===== 相机写入 JSON 的路径（相机端单位=米） =====
LATEST_PATH = "/home/robot/download/w/latest_point.json"  # Windows 可用 r"E:\\zuobiaozhuanhuan\\latest_point.json"

# ===== 你的 MDH 正解函数（确保可导入） =====
# from estun_mdh_fk import frame1_to_base_coords


def read_latest_point_m(path: str, max_age_s: float = 10.0):
    """读取 JSON 相机点 (m)。无效/过期返回 None"""
    try:
        if not os.path.exists(path):
            return None
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        x, y, z = d.get("x_m"), d.get("y_m"), d.get("z_m")
        ts = float(d.get("ts", 0.0))
        if x is None or y is None or z is None:
            return None
        if (time.time() - ts) > max_age_s:
            return None
        return float(x), float(y), float(z)
    except Exception:
        return None


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

        # ===== 现场参数（与你之前一致，可按需修改）=====
        self.base_offset_at_zero = [81.5, 77.0, -7.0]  # q=0 时 TCP 在基坐标的偏移 (mm)
        self.ext_len = 230.0                           # 物体→TCP 连线取 230mm 点
        self.d6 = 138.5                                # 6轴法兰厚度 (mm)
        self.base_sign = (1, 1, 1)
        self.rot90k = 0

        self.gripper = None
        self.robotinfo = False

        self.object = []
        self.grip = 0

        # 期望的关节名（JointState 有 name 时用于重排）
        self.joint_names_expected = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
        ]

        # ---- RobotInfo 接入（优先）+ /joint_states（回退）----
        # self.robotinfo_topic = '/robotinfo'   # ←★按现场改成真实话题名
        self.ignore_non_running = False        # True: 仅 state=="AutoRunning" 才用
        self.ignore_fault = False             # True: fault_flag!=0 则忽略

        # 按你的格式：分行调用
        self.sub_robotinfo = self.create_subscription(
            RobotInfo,
            'RobotInfo',
            self.robot_info_callback,
            10
        )
        # self.sub_jointstate = self.create_subscription(
        #     JointState,
        #     '/joint_states',
        #     self.cb_joint_states,
        #     30
        # )
        # 订阅夹爪状态
        self.sub_gripper = self.create_subscription(
            GripperInput,
            'gripper_input',
            self.gripper_callback,
            10
        )

        # 发布夹爪期望位置
        self.pub_gripper = self.create_publisher(UInt8, 'gripper_goal', 10)

        # 发布位置坐标
        self.goal_xyz_point = self.create_publisher(PointStamped, '/target_point', 10)

        # 目标选择（默认 p230）
        self.target_choice = 'p230'
        self.sub_select = self.create_subscription(
            String,
            '/vision/target_select',
            self.cb_select,
            10
        )

        # 状态缓存
        self.q_rad = np.zeros(6, dtype=float)
        self.have_joints = False
        self.last_src = 'none'  # 'robotinfo' | 'joint_states'

        # 定时器
        self.timer = self.create_timer(
            21.5,
            self.cb_timer
        )

        self.get_logger().info("【测试版】EstunRealtimeFK 已启动：当前不发布任何话题，仅打印计算结果（米制）")
        self.get_logger().info(" /vision/target_select：p230（默认 p230）")

        # while not self.robotinfo:
        #     time.sleep(0.05)


    def robot_info_callback(self, msg):
        # 仍然保留原有缓存
        # self.robotinfo = msg
        try:
            if self.ignore_non_running and msg.state != "AutoRunning":
                self.robotinfo = True
                return
            else:
                self.robotinfo = False

            if self.ignore_fault and int(msg.fault_flag) != 0:
                return

            jp = np.asarray(msg.joint_positions, dtype=float)
            if jp.size >= 6:
                self.q_rad = jp[:6]  # RobotInfo 定义为弧度
                self.have_joints = True
                if self.last_src != 'robotinfo':
                    self.get_logger().info("关节源 = robotinfo/RobotInfo")
                    self.last_src = 'robotinfo'
        except Exception as e:
            self.get_logger().warn(f"RobotInfo 解析失败: {e}")

    def gripper_callback(self,msg2):
        self.gripper = msg2

    # ---------- 选择控制 ----------
    def cb_select(self, msg: String):
        v = (msg.data or "").strip().lower()
        if v in ('object', 'tcp', 'p230'):
            if v != self.target_choice:
                self.get_logger().info(f"切换目标选择: {self.target_choice} -> {v}")
            self.target_choice = v
        else:
            self.get_logger().warn(f"无效选择: {msg.data!r}，应为 object|tcp|p230")

    # ---------- RobotInfo 回调（优先） ----------
    def cb_robotinfo(self, msg: 'RobotInfo'):
        print("cb_robotinfo")
        try:
            if self.ignore_non_running and msg.state != "AutoRunning":
                return
            if self.ignore_fault and int(msg.fault_flag) != 0:
                return

            jp = np.asarray(msg.joint_positions, dtype=float)
            if jp.size < 6:
                return
            self.q_rad = jp[:6]  # RobotInfo 定义为弧度
            self.have_joints = True
            if self.last_src != 'robotinfo':
                self.get_logger().info("关节源 = robotinfo/RobotInfo")
                self.last_src = 'robotinfo'
        except Exception as e:
            self.get_logger().warn(f"RobotInfo 解析失败: {e}")

    # ---------- /joint_states 回调（回退） ----------
    def cb_joint_states(self, msg: JointState):
        try:
            if self.have_joints and self.last_src == 'robotinfo':
                return  # 已用 robotinfo 就不覆盖
            if not msg.position or len(msg.position) < 6:
                return

            if msg.name and len(msg.name) == len(msg.position):
                name_to_idx = {n: i for i, n in enumerate(msg.name)}
                q_list = []
                for n in self.joint_names_expected:
                    if n in name_to_idx:
                        q_list.append(msg.position[name_to_idx[n]])
                    else:
                        q_list = list(msg.position[:6])
                        break
                q = np.asarray(q_list[:6], dtype=float)
            else:
                q = np.asarray(msg.position[:6], dtype=float)

            self.q_rad = q  # JointState 一般也是弧度
            self.have_joints = True
            if self.last_src != 'joint_states':
                self.get_logger().info("关节源 = /joint_states（回退）")
                self.last_src = 'joint_states'
        except Exception as e:
            self.get_logger().warn(f"/joint_states 解析失败: {e}")

    # ---------- 主循环 ----------
    def cb_timer(self):
        if not self.have_joints:
            return

        # 1) 相机点（米）
        pt_m = read_latest_point_m(
            LATEST_PATH,
            max_age_s=10.0
        )
        if pt_m is None:
            return
        camera_x, camera_y, camera_z = pt_m
        p1_mm = np.array([camera_x, camera_y, camera_z]) * 1000.0  # m -> mm

        # 2) 相机系(1) -> 基坐标(2)（内部毫米）
        try:
            p2_mm, T0E, _ = frame1_to_base_coords(
                self.q_rad,
                p1_mm,
                base_offset_at_zero=self.base_offset_at_zero,
                d6=self.d6,
                base_sign=self.base_sign,
                rot90k=self.rot90k
            )
        except Exception as e:
            self.get_logger().error(f"FK 计算失败: {e}")
            return

        # 3) TCP 与 230mm 点（毫米）
        p_tcp_mm = T0E[:3, 3].reshape(3)
        p230_mm = point_on_line_from_object(
            p2_mm,
            p_tcp_mm,
            d_mm=self.ext_len
        )
        camera_m = np.array([camera_x, camera_y, camera_z], dtype=float)  # 相机系(米)
        object_m = np.asarray(p2_mm, dtype=float) / 1000.0  # 物体在基坐标(米)

        np.set_printoptions(precision=6, suppress=True)
        self.get_logger().info(f"[TEST] camera frame point [m] = {np.round(camera_m, 6)}")
        self.get_logger().info(f"[TEST] object in base [m]    = {np.round(object_m, 6)}")
        self.object = np.round(camera_m, 6)

        # 4) 选择目标 -> 转“米” -> 仅打印（不发布）

        target_mm = p230_mm

        target_m = np.asarray(target_mm, dtype=float) / 1000.0  # mm -> m
        np.set_printoptions(precision=6, suppress=True)

        # ===== 测试阶段仅打印 =====
        self.get_logger().info(
            f"[TEST] target({self.target_choice}) in base [m] = "
            f"{np.round(target_m, 6)}"
        )

        # ===== 未来要发布时，取消下方注释 =====
        # while self.robotinfo.state != "AutoReady":
        #     time.sleep(0.05)
        # self.publish_target_point()
        grip = 65
        grip_msg = UInt8()
        grip_msg.data = grip
        self.pub_gripper.publish(grip_msg)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.point.x = float(object_m[0]) - 0.01
        msg.point.y = float(object_m[1]) - 0.01
        msg.point.z = float(object_m[2]) + 0.20
        self.goal_xyz_point.publish(msg)
        time.sleep(4)

        grip = 228
        grip_msg = UInt8()
        grip_msg.data = grip
        self.pub_gripper.publish(grip_msg)
        time.sleep(1.8)

        msg1 = PointStamped()
        msg1.header.stamp = self.get_clock().now().to_msg()
        msg1.header.frame_id = 'base'
        msg1.point.x = msg.point.x
        msg1.point.y = msg.point.y
        msg1.point.z = msg.point.z + 0.14
        self.goal_xyz_point.publish(msg1)
        print("UP")
        time.sleep(4)

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
        grip_msg = UInt8()
        grip_msg.data = grip
        self.pub_gripper.publish(grip_msg)
        time.sleep(1.5)

        msg3 = PointStamped()
        msg3.header.stamp = self.get_clock().now().to_msg()
        msg3.header.frame_id = 'base'
        msg3.point.x = float(-0.3415)
        msg3.point.y = float(-0.0335)
        msg3.point.z = float(0.4285)
        self.goal_xyz_point.publish(msg3)
        print("UP")
        time.sleep(4)


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