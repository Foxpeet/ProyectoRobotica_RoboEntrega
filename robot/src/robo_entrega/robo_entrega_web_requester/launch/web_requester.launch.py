from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Lanza el nodo "web_requester"
    """
    return LaunchDescription([
        Node(
            package='robo_entrega_web_requester',
            executable='',
            output='screen'),
    ])