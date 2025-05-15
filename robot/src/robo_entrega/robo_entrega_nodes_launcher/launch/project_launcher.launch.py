from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (TimerAction, RegisterEventHandler, ExecuteProcess,)
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Lanza lo siguiente:
        Lanza todos los servicios de movimiento nav2
        Lanza el visualizador Rviz con la configuracion adecuada
        Lanza los nodos nav2_system y provide_map para poder mostrar en el visualizador automaticamente el mapa y la posicion inicial del robot
    """

    return LaunchDescription([
        
    ])