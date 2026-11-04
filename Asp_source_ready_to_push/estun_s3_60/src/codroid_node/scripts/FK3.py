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
        self.robotinfo = None
        self.object = []
        self.grip = 0
        
        # 添加执行状态控制
        self.is_executing = False
        self.current_object_index = 0

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
        self.robotinfo = msg
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

    def wait_for_robot_state(self, desired_state, timeout=30.0):
        """等待机器人达到指定状态"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if (hasattr(self.robotinfo, 'state') and 
                self.robotinfo.state == desired_state):
                return True
            time.sleep(0.1)
        self.get_logger().warn(f"等待状态 {desired_state} 超时")
        return False

    def execute_grasp_sequence(self, object_m, object_index):
        """执行单个物体的抓取序列"""
        self.get_logger().info(f"开始抓取第 {object_index + 1} 个物体")
        
        try:
            # 步骤1: 张开夹爪并移动到安全高度
            self.get_logger().info("步骤1: 张开夹爪并移动到安全高度")
            grip_msg = UInt8()
            grip_msg.data = 60  # 张开夹爪
            self.pub_gripper.publish(grip_msg)
            
            # 移动到物体上方的安全高度
            safe_height_msg = PointStamped()
            safe_height_msg.header.stamp = self.get_clock().now().to_msg()
            safe_height_msg.header.frame_id = 'base'
            safe_height_msg.point.x = float(object_m[0]) - 0.015
            safe_height_msg.point.y = float(object_m[1])
            
            # 修正高度计算：确保安全高度足够高
            safe_height = max(float(object_m[2]) + 0.25, 0.35)  # 至少35cm高度
            safe_height_msg.point.z = safe_height
            
            self.goal_xyz_point.publish(safe_height_msg)
            self.get_logger().info(f"移动到安全高度: z={safe_height:.3f}")
            
            # 等待机械臂到达安全位置
            if not self.wait_for_robot_state("AutoReady", timeout=10.0):
                self.get_logger().error("移动到安全高度超时")
                return False
            time.sleep(1.0)  # 额外稳定时间
            
            # 步骤2: 下降到抓取高度
            self.get_logger().info("步骤2: 下降到抓取高度")
            grasp_msg = PointStamped()
            grasp_msg.header.stamp = self.get_clock().now().to_msg()
            grasp_msg.header.frame_id = 'base'
            grasp_msg.point.x = float(object_m[0]) - 0.015
            grasp_msg.point.y = float(object_m[1])
            
            # 修正抓取高度：确保足够低以抓取物体
            grasp_height = max(float(object_m[2]) + 0.15, 0.25)  # 至少25cm，但比安全高度低
            grasp_msg.point.z = grasp_height
            
            self.goal_xyz_point.publish(grasp_msg)
            self.get_logger().info(f"下降到抓取高度: z={grasp_height:.3f}")
            
            # 等待机械臂到达抓取位置
            if not self.wait_for_robot_state("AutoReady", timeout=10.0):
                self.get_logger().error("下降到抓取高度超时")
                return False
            time.sleep(0.5)  # 额外稳定时间
            
            # 步骤3: 闭合夹爪抓取物体
            self.get_logger().info("步骤3: 闭合夹爪抓取物体")
            grip_msg.data = 228  # 闭合夹爪
            self.pub_gripper.publish(grip_msg)
            time.sleep(2.0)  # 等待夹爪完全闭合
            
            # 步骤4: 抬升物体到安全高度
            self.get_logger().info("步骤4: 抬升物体到安全高度")
            lift_msg = PointStamped()
            lift_msg.header.stamp = self.get_clock().now().to_msg()
            lift_msg.header.frame_id = 'base'
            lift_msg.point.x = float(object_m[0]) - 0.015
            lift_msg.point.y = float(object_m[1])
            lift_msg.point.z = safe_height  # 使用之前的安全高度
            
            self.goal_xyz_point.publish(lift_msg)
            self.get_logger().info("抬升物体到安全高度")
            
            # 等待机械臂抬升完成
            if not self.wait_for_robot_state("AutoReady", timeout=10.0):
                self.get_logger().error("抬升物体超时")
                return False
            time.sleep(1.0)
            
            # 步骤5: 移动到放置位置
            self.get_logger().info("步骤5: 移动到放置位置")
            place_msg = PointStamped()
            place_msg.header.stamp = self.get_clock().now().to_msg()
            place_msg.header.frame_id = 'base'
            place_msg.point.x = 0.0773
            place_msg.point.y = 0.3363
            place_msg.point.z = 0.3672
            
            self.goal_xyz_point.publish(place_msg)
            self.get_logger().info("移动到放置位置")
            
            # 等待机械臂到达放置位置
            if not self.wait_for_robot_state("AutoReady", timeout=15.0):
                self.get_logger().error("移动到放置位置超时")
                return False
            time.sleep(1.0)
            
            # 步骤6: 释放物体
            self.get_logger().info("步骤6: 释放物体")
            grip_msg.data = 0  # 释放物体
            self.pub_gripper.publish(grip_msg)
            time.sleep(1.5)
            
            # 步骤7: 回到安全位置
            self.get_logger().info("步骤7: 回到安全位置")
            home_msg = PointStamped()
            home_msg.header.stamp = self.get_clock().now().to_msg()
            home_msg.header.frame_id = 'base'
            home_msg.point.x = -0.3415
            home_msg.point.y = -0.0335
            home_msg.point.z = 0.4285
            
            self.goal_xyz_point.publish(home_msg)
            self.get_logger().info("回到安全位置")
            
            # 等待机械臂回到安全位置
            if not self.wait_for_robot_state("AutoReady", timeout=10.0):
                self.get_logger().error("回到安全位置超时")
                return False
            time.sleep(1.0)
            
            self.get_logger().info(f"✅ 第 {object_index + 1} 个物体抓取完毕")
            return True
            
        except Exception as e:
            self.get_logger().error(f"抓取序列执行失败: {e}")
            return False

    def cb_timer(self):
        if self.is_executing:
            self.get_logger().warn("上一个抓取序列仍在执行，跳过本次回调")
            return
            
        if not self.have_joints:
            self.get_logger().warn("没有关节数据，跳过抓取")
            return

        # 检查机器人状态
        if not hasattr(self.robotinfo, 'state') or self.robotinfo.state != "AutoReady":
            self.get_logger().warn("机器人未就绪，跳过抓取")
            return

        # 读取全部目标点
        objects = read_all_objects_m(LATEST_DETECTIONS_PATH, max_age_s=10.0)
        if not objects:
            self.get_logger().warn("未检测到可抓取目标")
            return

        self.get_logger().info(f"检测到 {len(objects)} 个目标，开始抓取序列")

        # 在单独的线程中执行抓取序列，避免阻塞定时器
        import threading
        self.is_executing = True
        grasp_thread = threading.Thread(
            target=self.execute_grasp_sequence_for_all,
            args=(objects,)
        )
        grasp_thread.daemon = True
        grasp_thread.start()

    def execute_grasp_sequence_for_all(self, objects):
        """在单独线程中执行所有物体的抓取序列"""
        try:
            for i, (camera_x, camera_y, camera_z) in enumerate(objects):
                self.get_logger().info(f"准备抓取第 {i+1} 个目标: ({camera_x:.3f}, {camera_y:.3f}, {camera_z:.3f})")

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

                # 3) 获取物体在基坐标系中的位置
                object_m = np.asarray(p2_mm, dtype=float) / 1000.0
                self.get_logger().info(f"物体在基坐标系中的位置: {object_m}")

                # 4) 执行抓取序列
                success = self.execute_grasp_sequence(object_m, i)
                if not success:
                    self.get_logger().error(f"第 {i+1} 个物体抓取失败，跳过后续物体")
                    break
                    
                self.get_logger().info(f"✅ 第 {i+1} 个物体抓取完毕，继续下一个...\n")
                
            self.get_logger().info("🎯 所有检测到的物体抓取完成。")
            
        except Exception as e:
            self.get_logger().error(f"抓取序列执行异常: {e}")
        finally:
            self.is_executing = False


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
