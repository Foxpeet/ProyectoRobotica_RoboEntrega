import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

class LoadMapClient(Node):
    def __init__(self):
        super().__init__('load_map_client')
        self.client = self.create_client(LoadMap, '/map_server/load_map')

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Esperando que el servicio /map_server/load_map esté disponible...')

        self.send_request()

    def send_request(self):
        req = LoadMap.Request()
        req.map_url = '/home/tu_usuario/Git/ProyectoRobotica_RoboEntrega/robot/src/provide_map/map/my_map.yaml'

        self.get_logger().info(f'Solicitando carga del mapa: {req.map_url}')
        future = self.client.call_async(req)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            if response.result == 1:
                self.get_logger().info('✅ Mapa cargado correctamente.')
            else:
                self.get_logger().error('❌ Error al cargar el mapa.')
        except Exception as e:
            self.get_logger().error(f'Error en la llamada al servicio: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LoadMapClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
