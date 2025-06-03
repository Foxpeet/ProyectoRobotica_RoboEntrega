import rclpy
from rclpy.node import Node
import requests
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class ApiClientNode(Node):

    def __init__(self):
        super().__init__('api_client_node')
        self.posicion_actual_x = 0
        self.posicion_actual_y = 0
        self.timer = self.create_timer(15.0, self.call_api)
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

        url = 'http://127.0.0.1:5000/api/entregas/nocompletadas/ubicaciones' 
        url_confirm = 'http://127.0.0.1:5000/api/entregas/completar/' 
        try:
            response = requests.get(url)
            response.raise_for_status()
            data = response.json()
            self.get_logger().info(f"Respuesta de la API: {data}")
            if (data['count'] == 0):
                return
            
            self.estoy_ocupado = True
            
            # calculamos la distancia desde el robot a todas las entregas
            # Aqui va la IA que decide el mejor a donde ir (teoria de grafos?)
            #for i in range(data['count']):
            #    self.get_logger().info(f"me {i}")

            id_entrega = data['data'][0]['id_entrega']

            ubi_destino = data['data'][0]['destino'] #comprobar si origen es null
            ubi_origen = data['data'][0]['origen']

            msg = String()
            msg.data = "Voy a hacer una entrega de {ubi_origen} a {ubi_destino}"
            self.publisher_.publish(msg)
            self.get_logger().info(f"mensaje enviado")
            
            # mandar solicitud a moverse a origen y esperar confirmacion para mandarlo a destino

            response_confirm = requests.put(url_confirm + str(id_entrega))
            msg = String()
            msg.data = "He terminado la entrega"
            self.publisher_.publish(msg)
            self.get_logger().info(f"mensaje enviado")
            # hacer comprobacion de confirmacion
            self.estoy_ocupado = False
            

        except requests.RequestException as e:
            self.get_logger().error(f"Error en la petición: {e}")

    def cambia_pos(self, msg):
        self.posicion_actual_x = msg.position.x
        self.posicion_actual_y = msg.position.y

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