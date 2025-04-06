from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robo_entrega_nav_to_pose',
            executable='nav_to_pose_subscriber',
            output='screen'),
    ])