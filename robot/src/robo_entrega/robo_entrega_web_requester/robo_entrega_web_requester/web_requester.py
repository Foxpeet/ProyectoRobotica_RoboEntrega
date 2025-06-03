import rclpy
from rclpy.node import Node
import requests
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from math import radians, sin, cos

class ApiClientNode(Node):

    def __init__(self):
        super().__init__('api_client_node')
        self.posicion_actual_x = 0
        self.posicion_actual_y = 0
        self.timer = self.create_timer(15.0, self.call_api)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.publisher_ = self.create_publisher(String, '/mensajes_admin', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.cambia_pos,
            10
        )
        self.estoy_ocupado = False

    def call_api(self):
        if self.estoy_ocupado == True:
            return

        self.url = 'http://127.0.0.1:5000/api/entregas/nocompletadas/ubicaciones' 
        self.url_confirm = 'http://127.0.0.1:5000/api/entregas/completar/' 
        try:
            response = requests.get(self.url)
            response.raise_for_status()
            data = response.json()
            self.get_logger().info(f"Respuesta de la API: {data}")
            if (data['count'] == 0):
                return
            
            self.estoy_ocupado = True
            
            # calculamos la distancia desde el robot a todas las entregas
            # Aqui va la IA que decide el mejor a donde ir (teoria de grafos?)
            distancia_menor = 9999
            pos_entrega_objetivo = 0
            for i in range(data['count']):
                # longitud == x || latitud == y
                distancia_origen = math.sqrt((data['data'][i]['origen']['longitud'] - self.posicion_actual_x)**2 + (data['data'][i]['origen']['latitud'] - self.posicion_actual_y)**2)
                distancia_destino = math.sqrt((data['data'][i]['destino']['longitud'] - data['data'][i]['origen']['longitud'])**2 + (data['data'][i]['destino']['latitud'] - data['data'][i]['origen']['latitud'])**2)
                distancia_total = distancia_origen + distancia_destino

                if distancia_menor > distancia_total:
                    distancia_menor = distancia_total
                    pos_entrega_objetivo = 0

            self.id_entrega = data['data'][pos_entrega_objetivo]['id_entrega']
            self.get_logger().info(f"id entrega {self.id_entrega}")

            self.ubi_destino = data['data'][pos_entrega_objetivo]['destino'] #comprobar si origen es null
            ubi_origen = data['data'][pos_entrega_objetivo]['origen']

            self._action_client.wait_for_server()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = ubi_origen['longitud']
            goal_msg.pose.pose.position.y = ubi_origen['latitud']
            w_rad = 0

            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)

            msg = String()
            msg.data = f"Voy a hacer una entrega de {ubi_origen['nombre']} para {self.ubi_destino['nombre']} \n"
            self.publisher_.publish(msg)
            self.get_logger().info(f"mensaje enviado")
            
        except requests.RequestException as e:
            self.get_logger().error(f"Error en la petición: {e}")

    def cambia_pos(self, msg):
        self.posicion_actual_x = msg.pose.pose.position.x
        self.posicion_actual_y = msg.pose.pose.position.y

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rechazado')
            self.estoy_ocupado = False
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navegación completada con estado: {result}')

        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.ubi_destino['longitud']
        goal_msg.pose.pose.position.y = self.ubi_destino['latitud']
        w_rad = 0

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback_destino)

    def goal_response_callback_destino(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rechazado')
            self.estoy_ocupado = False
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback_destino)
    
    def get_result_callback_destino(self, future):
        result = future.result().result
        self.get_logger().info(f'Navegación completada con estado: {result}')

        response_confirm = requests.put(self.url_confirm + str(self.id_entrega))
        msg = String()
        msg.data = "He terminado la entrega \n"
        self.publisher_.publish(msg)
        self.get_logger().info(f"mensaje enviado")
        
        self.estoy_ocupado = False


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)
    node = ApiClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()