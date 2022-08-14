from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atlas_basic',
            executable='wheel_control'
        ),
        Node(
            package='atlas_basic',
            executable='odometry'
        ),
        Node(
            package='atlas_basic',
            executable='planner'
        )
    ])