import rclpy
from rclpy.node import Node
import requests

class ApiClientNode(Node):

    def __init__(self):
        super().__init__('api_client_node')
        self.timer = self.create_timer(10.0, self.call_api)
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
            
            # Aqui va la IA que decide el mejor a donde ir (teoria de grafos?)
            id_entrega = data['data'][0]['id_entrega']

            ubi_destino = data['data'][0]['destino'] #comprobar si origen es null
            ubi_origen = data['data'][0]['origen']
            
            # mandar solicitud a moverse a origen y esperar confirmacion para mandarlo a destino

            response_confirm = requests.put(url_confirm + str(id_entrega))
            # hacer comprobacion de confirmacion
            self.estoy_ocupado = False
            

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