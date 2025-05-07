import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import math

class MyWaypointFollower(Node):

    def __init__(self):
        super().__init__('my_waypoint_follower')
        
        # Action client para enviar waypoints
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._goal_handle = None  # Para guardar la referencia al goal actual
        
        # Suscriptor para detectar tareas externas (ej. desde RViz)
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Topic usado por Nav2 para navegación manual
            self._goal_callback,
            10
        )
        
        # Timer para enviar waypoints en bucle
        self._timer = self.create_timer(1.0, self._send_goal_if_ready)
        self._goal_sent = False
        self._other_task_active = False  # Bandera para evitar conflictos

    def _goal_callback(self, msg):
        """Callback para detectar nuevas tareas externas (ej. clics en RViz)."""
        if self._goal_handle is not None:
            # Cancelar la ruta actual si hay una activa
            self.get_logger().info('Tarea externa detectada, cancelando waypoints actuales...')
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._cancel_done_callback)
        self._other_task_active = True

    def _cancel_done_callback(self, future):
        """Callback cuando se completa la cancelación."""
        if future.result().return_code == 1:  # 1 = éxito
            self.get_logger().info('Waypoints cancelados correctamente')
        self._goal_handle = None
        self._goal_sent = False
        # No resetear _other_task_active aquí, se hará cuando termine la tarea externa

    def _send_goal_if_ready(self):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor follow_waypoints...')
            return

        if not self._goal_sent and not self._other_task_active:
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = self._create_waypoints()
            self.get_logger().info('Enviando lista de waypoints...')
            
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self._goal_response_callback)
            self._goal_sent = True

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('La ruta fue rechazada')
            self._goal_sent = False
            return

        self.get_logger().info('Ruta aceptada, navegando...')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Waypoints completados. Reiniciando bucle...')
        self._goal_sent = False
        self._other_task_active = False  # Permitir nuevo ciclo de waypoints
        self._goal_handle = None

    def _create_waypoints(self):
        waypoints = []
        positions = [
            (0.0, 0.0, 0.0),
            (1.5, -5.0, 270.0),
            (0.0, -22.0, 0.0),
            (4.0, -22.0, 90.0),
            (5.0, -6.0, 0.0),
            (9.5, -6.5, 270.0),
            (8.5, -22.5, 0.0),
            (13.0, -22.5, 90.0),
            (14.0, -6.5, 180.0),
        ]
        for x, y, yaw in positions:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            waypoints.append(pose)
        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = MyWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()