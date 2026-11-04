import os
import launch
import xacro
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    # Package directory
    pkg_dir = get_package_share_directory('s3_60_eco_2024')

    # URDF file path - use the fixed version
    urdf_file = os.path.join(pkg_dir, 'urdf', 's3_60_eco_2024.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as file:
        urdf_contents = file.read()

    # Joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'robot_description': urdf_contents
        }]
    )
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_contents,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0
        }]
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'urdf.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return [joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz2_node]

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        OpaqueFunction(function=launch_setup)
    ]) 