from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='codroid_node',
            executable='codroid_io.py',
            name='codroid_io',
            output='screen',
            parameters=[]
        ),
        Node(
            package='codroid_node',
            executable='codroid_move_test.py',
            name='codroid_move_test',
            output='screen',
            parameters=[]
        )
    ])
