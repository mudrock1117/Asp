
将以下内容复制到文件中：
python

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestMotion(Node):
    def __init__(self):
        super().__init__('test_motion')
        
        # 创建发布器
        self.publisher = self.create_publisher(String, '/test_topic', 10)
        
        # 创建定时器
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info("TestMotion节点已启动")

    def timer_callback(self):
        """定时器回调函数"""
        self.counter += 1
        
        msg = String()
        msg.data = f"测试消息 #{self.counter}"
        
        self.publisher.publish(msg)
        self.get_logger().info(f"发布: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = TestMotion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

3. 修改 package.xml 文件
bash

# 编辑package.xml
nano ../package.xml

在 package.xml 中添加或确保包含以下依赖：
xml

<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypelayout="1.0"?>
<package format="3">
  <name>codroid_node</name>
  <version>0.0.0</version>
  <description>Codroid robot control nodes</description>
  <maintainer email="robot@todo.todo">robot</maintainer>
  <license>Apache-2.0</license>

  <!-- 基本依赖 -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <!-- 消息依赖 -->
  <exec_depend>codroid_msgs</exec_depend>
  <exec_depend>robotiq_interfaces</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

4. 修改 setup.py 文件
bash

# 编辑setup.py
nano ../setup.py

在 setup.py 的 entry_points 部分添加新节点：
python

from setuptools import setup

package_name = 'codroid_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Codroid robot control nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 已有的节点
            'fk.py = codroid_node.fk:main',
            'MotionPlanner.py = codroid_node.MotionPlanner:main',
            'codroid_move_test.py = codroid_node.codroid_move_test:main',
            'Robotiq2FGripperSimpleController.py = codroid_node.Robotiq2FGripperSimpleController:main',
            
            # 新添加的节点
            'testMotion.py = codroid_node.testMotion:main',
        ],
    },
)

5. 重新编译包
bash

# 回到工作空间根目录
cd ~/estun_s3_60

# 重新编译包
colcon build --packages-select codroid_node

# 加载环境
source install/setup.bash

6. 测试新节点
bash

# 测试运行新节点
ros2 run codroid_node testMotion.py

# 在另一个终端查看节点信息
ros2 node list
ros2 node info /test_motion

# 查看发布的topic
ros2 topic echo /test_topic

如果 setup.py 不存在的情况

如果您的包中没有 setup.py 文件，只有 CMakeLists.txt，那么需要修改 CMakeLists.txt：
bash

# 编辑CMakeLists.txt
nano ../CMakeLists.txt

在 CMakeLists.txt 中添加：
cmake

# 安装Python脚本
install(PROGRAMS
  scripts/fk.py
  scripts/MotionPlanner.py
  scripts/codroid_move_test.py
  scripts/Robotiq2FGripperSimpleController.py
  scripts/testMotion.py  # 添加新节点
  DESTINATION lib/${PROJECT_NAME}
)

完整的一键创建脚本

如果您想快速创建，可以使用这个一键脚本：
bash

#!/bin/bash
# create_new_node.sh

NODE_NAME=$1

if [ -z "$NODE_NAME" ]; then
    echo "使用方法: ./create_new_node.sh <节点文件名>"
    exit 1
fi

cd ~/estun_s3_60/src/codroid_node/scripts

# 创建节点文件
cat > $NODE_NAME.py << 'EOF'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        self.publisher = self.create_publisher(String, '/test_topic', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info("测试节点已启动")

    def timer_callback(self):
        self.counter += 1
        msg = String()
        msg.data = f"消息 #{self.counter}"
        self.publisher.publish(msg)
        self.get_logger().info(f"发布: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# 设置权限和格式
chmod +x $NODE_NAME.py
dos2unix $NODE_NAME.py

echo "节点 $NODE_NAME.py 创建成功"
echo "请手动修改 setup.py 或 CMakeLists.txt 来注册新节点"

使用方式：
bash

chmod +x create_new_node.sh
./create_new_node.sh testMotion

按照这些步骤，您就可以成功在 codroid_node 包中创建并运行新的 ROS2 节点了。
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
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
import time


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

        # 目标点相关变量
        self.target_point = None
        self.target_frame = "base_link"  # 默认坐标系
        # 异步任务相关
        self.current_move_task = None

        self.current_joint_state = None
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

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

    async def target_point_callback(self, msg):
        """处理接收到的目标坐标"""
        try:
            self.target_point = msg.point
            self.target_frame = msg.header.frame_id

            # 确保消息字段是浮点数类型
            # if not hasattr(msg, 'point') or not hasattr(msg.point, 'x') or not hasattr(msg.point, 'y') or not hasattr(
            #         msg.point, 'z'):
            #     self.get_logger().error("消息格式错误，缺少必要的字段")
            #     return

            # 显式转换为浮点数，确保类型正确
            # x = float(msg.point.x)
            # y = float(msg.point.y)
            # z = float(msg.point.z)

            # 创建新的Point对象，确保所有字段都是浮点数
            # self.target_point = Point()
            # self.target_point.x = x
            # self.target_point.y = y
            # self.target_point.z = z

            # self.target_frame = msg.header.frame_id if hasattr(msg.header, 'frame_id') else "base_link"

            self.get_logger().info(
                f"收到目标坐标 - 坐标系: {self.target_frame}, "
                f"位置: ({self.target_point.x:.3f}, {self.target_point.y:.3f}, {self.target_point.z:.3f})"
            )
            # test_pose = self.create_pose(self.target_point.x, self.target_point.y, self.target_point.z, 180.0 / 180.0 * pi, -0.0 / 180.0 * pi,
            #                              -90.0 / 180.0 * pi)
            test_pose = self.create_pose(0.326764, 0.109782, 0.386671, 180.0 / 180.0 * pi, -0.0 / 180.0 * pi,
                                         -90.0 / 180.0 * pi)
            # 使用您提供的精确约束条件
            joint_constraints = {
                "joint2": {
                    "position": -0.8,
                    "tolerance_above": 1.0,
                    "tolerance_below": 1.0
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
                    "tolerance_above": 1.65,
                    "tolerance_below": 1.65
                }
            }
            # 修复：使用asyncio.create_task()而不是直接await
            # if self.current_move_task is None or self.current_move_task.done():
            #     self.current_move_task = asyncio.create_task(
            #         self.execute_move_to_pose(test_pose, joint_constraints)
            #     )
            # else:
            #     self.get_logger().warning("当前有移动任务正在执行，忽略新目标")

            # 移除 create_task，直接使用 await（需要将回调函数改为异步）
            success = await self.move_to_pose(test_pose, joint_constraints)
            self.get_logger().info(f"测试复杂笛卡尔位姿: {'成功' if success else '失败'}")

        except Exception as e:
            self.get_logger().error(f"处理目标坐标时发生错误: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    # 添加一个新的异步方法来处理移动执行和结果记录
    # async def execute_move_to_pose(self, test_pose, joint_constraints):
    #      """执行移动并记录结果"""
    #      try:
    #          self.get_logger().info("调用 move_to_pose...")
    #          success = await asyncio.wait_for(
    #              self.move_to_pose(test_pose, joint_constraints),
    #              timeout=30.0  # 30秒超时
    #          )
    #          self.get_logger().info(f"move_to_pose 返回: {success}")
    #      except Exception as e:
    #          self.get_logger().error(f"执行移动时发生错误: {e}")
    #          return False

    async def move_to_joints(self, joint_positions):
        """移动到指定关节角度"""
        return await self._send_joint_move_request(joint_positions=joint_positions)

    async def move_to_pose(self, pose, joint_constraints=None):
        """移动到指定位姿（先使用逆运动学求解，再使用关节空间规划）"""
        # 使用逆运动学将笛卡尔位姿转换为关节角度
        joint_target = await self._solve_ik(pose, joint_constraints)

        # 修复4: 移除错误的代码，简化逻辑
        if joint_target is not None:
            # self.get_logger().info(f"逆运动学求解结果: {[f'{pos:.3f}' for pos in joint_target]}")
            print(joint_target)
            return await self._send_joint_move_request(joint_positions=joint_target)
        else:
            self.get_logger().error("逆运动学求解失败")
            return False

    async def _send_joint_move_request(self, joint_positions):
        """发送关节空间移动请求到MoveGroup"""
        # 检查是否有当前关节状态
        if self.current_joint_state is None:
            self.get_logger().error("没有当前关节状态，无法规划")
            return False

        if not isinstance(joint_positions, (list, tuple)):
            self.get_logger().error(f"关节位置类型错误: {type(joint_positions)}")
            return False

        if len(joint_positions) != len(self.joint_names):
            self.get_logger().error(
                f"关节位置数量不匹配: 期望{len(self.joint_names)}, 实际{len(joint_positions)}"
            )
            self.get_logger().error(f"关节名称: {self.joint_names}")
            self.get_logger().error(f"收到位置: {joint_positions}")
            return False

            # 检查关节值是否有效（非NaN、在合理范围内）
        for i, pos in enumerate(joint_positions):
            if not isinstance(pos, (int, float)):
                self.get_logger().error(f"关节{i}位置类型错误: {type(pos)}")
                return False
            if math.isnan(pos) or math.isinf(pos):
                self.get_logger().error(f"关节{i}位置无效: {pos}")
                return False

            # 原有的其余检查...
        if self.current_joint_state is None:
            self.get_logger().error("没有当前关节状态，无法规划")
            return False

        # 创建MoveGroup目标
        goal_msg = MoveGroup.Goal()

        # 设置规划参数
        goal_msg.request.group_name = "estun"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 2.0
        goal_msg.request.max_velocity_scaling_factor = 0.8
        goal_msg.request.max_acceleration_scaling_factor = 0.6
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
            response = await future

            if response.accepted:
                self.get_logger().info("移动请求已被接受")
                result_future = response.get_result_async()
                result = await result_future

                if result.status == 4:  # SUCCEEDED
                    self.get_logger().info("移动执行成功!")
                    return True
                else:
                    self.get_logger().error(f"移动执行失败，状态码: {result.status}")
                    return False
            else:
                self.get_logger().error("移动请求被拒绝")
                return False

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
        # request.ik_request.timeout.sec = 3
        # request.ik_request.attempts = 5

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
        self.get_logger().info("IK请求已发送，等待响应...")

        # 等待响应
        # while rclpy.ok():
        #     rclpy.spin_once(self)
        #     print("ready")
        #     if future.done():
        #         try:
        #             response = future.result()
        #         except Exception as e:
        #             self.get_logger().info(f'IK服务调用失败: {e}')
        #             return None
        #         else:
        #             self.get_logger().info('IK解算成功!')
        #             # 处理响应，例如打印关节位置
        #             if response.solution.joint_state.position:
        #                 self.get_logger().info(
        #                     f'解算结果: {response.solution.joint_state.position}')
        #                 return response.solution.joint_state.position
        #         break
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

        # return None

        # return response.solution.joint_state.position

    # def _build_complete_joint_positions(self, ik_names, ik_positions):
    #     """根据IK返回的关节数据构建完整的6关节位置数组"""
    #     # 初始化完整关节位置（使用当前状态或默认值）
    #     if self.current_joint_state and len(self.current_joint_state.position) == len(self.joint_names):
    #         complete_positions = list(self.current_joint_state.position)
    #         self.get_logger().info("使用当前关节状态作为基础")
    #     else:
    #         complete_positions = [0.0] * len(self.joint_names)
    #         self.get_logger().info("使用默认零位置作为基础")
    #
    #     # 用IK解算结果更新对应的关节
    #     updated_count = 0
    #     for i, name in enumerate(ik_names):
    #         if name in self.joint_names:
    #             idx = self.joint_names.index(name)
    #             complete_positions[idx] = ik_positions[i]
    #             updated_count += 1
    #             self.get_logger().info(f"更新关节 {name}[{idx}] = {ik_positions[i]:.3f}")
    #
    #     self.get_logger().info(f"成功更新了 {updated_count}/{len(ik_names)} 个关节")
    #
    #     # 验证最终结果
    #     if len(complete_positions) != len(self.joint_names):
    #         self.get_logger().error(f"最终关节位置数量错误: {len(complete_positions)} != {len(self.joint_names)}")
    #         return None
    #
    #     return complete_positions

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
    # node.get_logger().info("等待关节状态信息...")
    # import time
    # start_time = time.time()
    # while node.current_joint_state is None and (time.time() - start_time) < 10.0:
    #     rclpy.spin_once(node, timeout_sec=0.1)

    # if node.current_joint_state is None:
    #     node.get_logger().error("在超时时间内未收到关节状态信息")
    #     node.destroy_node()
    #     rclpy.shutdown()
    #     return

    # node.get_logger().info("节点已就绪，输入命令:")
    # node.get_logger().info("1 - 回零 (关节空间)")
    # node.get_logger().info("2 - 测试位置 (关节空间)")
    # node.get_logger().info("3 - 测试位姿 (先逆运动学求解，再关节空间规划)")
    # node.get_logger().info("4 - 测试简单位姿 (与ik.py相同的位姿)")
    # node.get_logger().info("q - 退出")

    try:
        # node.get_logger().info("运动规划器已启动，等待目标点...")
        # 简单的异步主循环
        # while rclpy.ok():
        rclpy.spin(node)
        # 处理ROS事件，但不要阻塞
        # rclpy.spin_once(node, timeout_sec=0.1)

        # 让出控制权给其他异步任务
        # await asyncio.sleep(0.01)
        # while rclpy.ok():
        # 简单的命令行交互
        # user_input = input("请输入命令: ").strip()
        #
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
        #
        # elif user_input == '3':
        # 测试复杂笛卡尔位姿
        # test_pose = node.create_pose(0.326764, 0.109782, 0.286671, 180.0 / 180.0 * pi, -0.0 / 180.0 * pi,
        #                              -90.0 / 180.0 * pi)
        # test_pose = node.create_pose(-0.044998, 0.254614, 0.0036915, 180.0 / 180.0 * pi, -0.0 / 180.0 * pi,
        #            -90.0 / 180.0 * pi)
        # 使用您提供的精确约束条件
        # joint_constraints = {
        #     "joint2": {
        #         "position": -0.8,
        #         "tolerance_above": 1.0,
        #         "tolerance_below": 1.0
        #      },
        #      "joint3": {
        #          "position": 1.6,
        #          "tolerance_above": 1.8,
        #          "tolerance_below": 1.8
        #      },
        #     "joint4": {
        #         "position": 0.0,
        #         "tolerance_above": 1.2,
        #         "tolerance_below": 1.58
        #     },
        #     "joint5": {
        #         "position": 1.57,
        #         "tolerance_above": 1.65,
        #         "tolerance_below": 1.65
        #     }
        # }
        # success = await node.move_to_pose(test_pose, joint_constraints)
        # node.get_logger().info(f"测试复杂笛卡尔位姿: {'成功' if success else '失败'}")

        # elif user_input == 'q':
        #     break
        # else:
        #     node.get_logger().info("未知命令")

        # 处理ROS事件
        # rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
