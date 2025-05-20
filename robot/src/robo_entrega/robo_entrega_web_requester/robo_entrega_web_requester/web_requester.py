import rclpy
from rclpy.node import Node
import requests

class ApiClientNode(Node):

    def __init__(self):
        super().__init__('api_client_node')
        self.timer = self.create_timer(10.0, self.call_api)

    def call_api(self):
        url = 'https://127.0.0.1:5000/api/' 
        try:
            response = requests.get(url)
            response.raise_for_status()
            data = response.json()
            self.get_logger().info(f"Respuesta de la API: {data}")
        except requests.RequestException as e:
            self.get_logger().error(f"Error en la petición: {e}")

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