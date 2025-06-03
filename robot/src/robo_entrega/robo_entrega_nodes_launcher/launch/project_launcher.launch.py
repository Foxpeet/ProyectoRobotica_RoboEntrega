from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (TimerAction, RegisterEventHandler, ExecuteProcess,IncludeLaunchDescription)
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Lanza lo siguiente:
        Lanza todos los nodos que tratan de la navegacion, la deteccion de imagen y el visualizador rviz2
    """

    nav2_yaml = os.path.join(get_package_share_directory('robo_entrega_nav2_system'), 'config', 'my_nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('robo_entrega_nav2_system'), 'config', 'my_map.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('robo_entrega_nav2_system'), 'config', 'map.rviz')

    #nodo_nav2_system/tb3
    initial_pose_launch = ExecuteProcess(
        cmd=['ros2', 'run', 'robo_entrega_nav2_system', 'initial_pose_pub', '--ros-args', '-p', 'use_sim_time:=True']
    )
    load_map_launch = ExecuteProcess(
        cmd=['ros2', 'run', 'robo_entrega_provide_map', 'load_map_client']
    )
    #capture_image / nodos
    launch_capture_image = ExecuteProcess(
        cmd=['ros2', 'run', 'robo_entrega_capture_image', 'detectar_caja']
    )
    #waypoints
    launch_waypoints = ExecuteProcess(
        cmd=['ros2', 'run', 'robo_entrega_nav2_system', 'my_waypoint_follower']
    )
    
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

        TimerAction(
            period=10.0,
            actions=[load_map_launch]
        ),

        RegisterEventHandler(
            OnProcessStart(
                target_action=load_map_launch,
                on_start=[initial_pose_launch]
            )
        ),

        # --- capture_image ---
        RegisterEventHandler(
            OnProcessStart(
                target_action=initial_pose_launch,
                on_start=[launch_capture_image]
            )
        ),
    ])