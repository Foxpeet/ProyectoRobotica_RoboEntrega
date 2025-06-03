from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Lanza lo siguiente:
        Lanza todos los servicios de movimiento nav2
        Lanza el visualizador Rviz con la configuracion adecuada
        Lanza los nodos nav2_system y provide_map para poder mostrar en el visualizador automaticamente el mapa y la posicion inicial del robot
    """

    nav2_yaml = os.path.join(get_package_share_directory('robo_entrega_nav2_system'), 'config', 'my_nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('robo_entrega_nav2_system'), 'config', 'my_map.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('robo_entrega_nav2_system'), 'config', 'map.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, {'yaml_filename': map_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        TimerAction(
            period=20.0,  # Esperar 10 segundos
            actions=[Node(
                package='robo_entrega_nav2_system',
                executable='initial_pose_pub',
                name='initial_pose_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )]
        ),
        TimerAction(
            period=15.0,  # Esperar 6 segundos
            actions=[Node(
                package='robo_entrega_provide_map',
                executable='load_map_client',
                name='load_map_client',
                output='screen'
            )]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': [
                            'map_server',
                            'amcl',
                            'planner_server',
                            'controller_server',
                            'recoveries_server',
                            'bt_navigator',
                            'waypoint_follower'
                        ]}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
    ])