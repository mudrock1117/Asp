from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("s3_60_eco_2024", package_name="estun_moveit2_config").to_moveit_configs()
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 获取URDF文件所在包的路径
    # 假设URDF文件在名为 "s3_60_eco_2024_description" 的包中
    urdf_package = "s3_60_eco_2024"  # 请替换为实际的包名
    urdf_package_path = get_package_share_directory(urdf_package)
    
    
    # 创建启动描述
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
    ])
    
    # 使用generate_move_group_launch函数
    move_group_launch = generate_move_group_launch(moveit_config)
    return generate_move_group_launch(moveit_config)
