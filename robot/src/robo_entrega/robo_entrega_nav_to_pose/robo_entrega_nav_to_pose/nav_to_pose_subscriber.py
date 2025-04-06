import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from math import radians, sin, cos

class NavigateClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')

        # Cliente de acción para NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Suscriptor al tópico /navigate_goal (x, y, yaw)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/navigate_goal',
            self.goal_callback,
            10
        )
        self.get_logger().info('Nodo listo y escuchando en /navigate_goal')

    def goal_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().error('El mensaje debe tener exactamente 3 valores: x, y, yaw')
            return
        x, y, yaw_deg = msg.data
        self.send_goal(x, y, yaw_deg)

    def send_goal(self, x, y, yaw_deg):
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Convertir yaw de grados a orientación quaternion (solo Z y W)
        yaw_rad = radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = cos(yaw_rad / 2.0)

        self.get_logger().info(f'Enviando Goal: x={x}, y={y}, yaw={yaw_deg}°')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rechazado por el servidor')
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navegación completada con estado: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


def main(args=None):
    rclpy.init(args=args)
    subscriber = NavigateClient()
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()