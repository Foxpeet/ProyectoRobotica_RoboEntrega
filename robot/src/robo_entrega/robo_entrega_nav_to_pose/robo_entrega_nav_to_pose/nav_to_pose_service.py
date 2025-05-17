import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from math import radians, sin, cos
from robo_entrega_custom_interface.srv import NavPoseSrv

"""
Este es el cliente del nodo "nav_to_pose" que escuchara el topic 
que venga de la web o el request que venga del robot para decirle al robot a que coordenadas del mapa moverse

Classes:
    NavToPoseService
    
"""

class NavToPoseService(Node):
    """
    Al llamarse a este nodo empezará un servicio que escucha el topic /navigate_goal
    que contiene un dato tipo std_msgs/msg/Float32MultiArray con 
    las coordenadas en metros X e Y y la orientacion en grados W,
    posteriormente pasa esos datos a un Goal y se lo envia a la acción
    NavigateToPose ya creada con el propio nav2_msgs de ros2.

    Methods:
        goal_callback(): Se llama cuando escucha un mensaje por el topic /navigate_goal al que se ha suscrito en __init__()
        goal_callback(): Se llama cuando recibe un request por el servicio nav_to_pose que se ha creado en __init__()
        send_goal(): Se llama una vez cuando se ha recibido los datos del topic o del servicio y mete los datos en un Goal para mandarlos a la acción
    """

    def __init__(self):
        super().__init__('nav_to_pose_service')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/navigate_goal',
            self.goal_callback,
            10
        )
        self.srv = self.create_service(NavPoseSrv, 'nav_to_pose_srv', self.service_callback)
        self.get_logger().info('Nodo listo y escuchando en /navigate_goal')

    def goal_callback(self, msg):
        self.check_topic_values(msg)
        x, y, w_deg = msg.data
        self.send_goal(x, y, w_deg)

    def service_callback(self, request, response):
        try:
            self.check_topic_values(request)
            x, y, w_deg = request.data

            self._action_client.wait_for_server()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y

            w_rad = radians(w_deg)
            goal_msg.pose.pose.orientation.z = sin(w_rad / 2.0)
            goal_msg.pose.pose.orientation.w = cos(w_rad / 2.0)
            self.get_logger().info(f"Mando goal")

            goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            goal_future.add_done_callback(self.goal_response_callback)
            while not goal_future.done():
                rclpy.spin_once(self, timeout_sec=3)
                self.get_logger().info(f'Goal1 {goal_future.result()}')

            goal_handle = goal_future.result()

            self.get_logger().info('Goal aceptado, esperando resultado...')
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            response.success = True
            return response
        except Exception as e:
            self.get_logger().error(f'Error en el servicio: {e}')
            response.success = False
        return response

    def send_goal(self, x, y, w_deg):
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        w_rad = radians(w_deg)
        goal_msg.pose.pose.orientation.z = sin(w_rad / 2.0)
        goal_msg.pose.pose.orientation.w = cos(w_rad / 2.0)

        self.get_logger().info(f'Enviando Goal: x={x}, y={y}, w={w_deg}°')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rechazado')
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navegación completada con estado: {result}')
        self._pending_response.success = True
        self.get_logger().info(f'Navegación servicio: {self._pending_response.success}')
        return self._pending_response

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    # --- Checkeo de excepciones ---
    def check_topic_values(self, msg):
        x, y, w_deg = msg.data
        if len(msg.data) != 3:
            raise ValueError('El mensaje debe tener exactamente 3 valores: x, y, w')
        if (type(x) is not float) or (type(y) is not float) or (type(w_deg) is not float):
            raise ValueError('Todos los componentes del mensaje deben ser tipo float')


def main(args=None):
    rclpy.init(args=args)
    service = NavToPoseService()
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()