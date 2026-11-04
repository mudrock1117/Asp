import time

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from time import sleep

def generate_launch_description():
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 添加夹爪设备路径参数
    gripper_device = LaunchConfiguration('gripper_device', default='/dev/ttyUSB0')
    
    # 获取包路径
    moveit_config_package = "estun_moveit2_config"
    moveit_config_path = get_package_share_directory(moveit_config_package)

    # 获取 robotiq 夹爪包路径
    robotiq_package = "robotiq"  # 请根据实际包名调整
    robotiq_path = get_package_share_directory(robotiq_package)
    
    # 获取 codroid_node 包路径
    codroid_node_package = "codroid_node"
    codroid_node_path = get_package_share_directory(codroid_node_package)
    
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'gripper_device',
            default_value='/dev/ttyUSB0',
            description='Device path for Robotiq gripper (e.g., /dev/ttyUSB0)'
        ),
    ])
    
    # 1. 启动 CodroidIO 节点（真实机械臂控制器）
    codroid_io_node = Node(
        package='codroid_node',
        executable='codroid_io.py',
        name='codroid_io',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(codroid_io_node)
    
    # 2. 启动轨迹处理节点
    codroid_move_test_node = Node(
        package='codroid_node',
        executable='codroid_move_testpp.py',
        name='codroid_move_test',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(codroid_move_test_node)

    # 2.1. 启动直接轨迹处理节点
    codroid_move_test_node1 = Node(
        package='codroid_node',
        executable='codroid_move_test.py',
        name='codroid_move',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(codroid_move_test_node1)
    
    # 3. 修改后的 demo 启动 - 使用真实控制器配置
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_config_path,
                'launch',
                'demo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'moveit_manage_controllers': 'true',  # 不让 MoveIt 管理控制器
            # 强制指定规划管道和参数文件
            'planning_pipeline': 'ompl',
 
        }.items()
    )
    ld.add_action(demo_launch)

    # 3. 启动 Robotiq 2F 夹爪节点
    robotiq_gripper_node = Node(
        package=robotiq_package,
        executable='robotiq_2f_gripper_node',  # 请根据实际文件名调整
        name='robotiq_2f_gripper_node',
        arguments=[gripper_device],  # 传递设备路径参数
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(robotiq_gripper_node)
    time.sleep(0.5)
    # 4. 启动夹爪简单控制器节点
    gripper_simple_controller_node = Node(
        package='robotiq',
        executable='gripper_simple_controller',  # 使用 setup.py 中注册的名称
        name='gripper_simple_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(gripper_simple_controller_node)
    
    # 5. 代码形式的规划器文件MotionPlanner，与codroid_node同包
    # motion_planner_node = Node(
    #     package='codroid_node',  # 假设 MotionPlanner.py 在 codroid_node 包中
    #     executable='MotionPlanner.py',
    #     name='simple_motion_planner',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    # ld.add_action(motion_planner_node)
    
    return ld
