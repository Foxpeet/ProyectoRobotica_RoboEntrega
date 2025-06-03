from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Lanza el nodo "nav_to_pose_service" que se quedará abierto escuchando el topic /navigation_goal
    """
    return LaunchDescription([
        Node(
            package='robo_entrega_nav_to_pose',
            executable='nav_to_pose_service',
            output='screen'),
    ])