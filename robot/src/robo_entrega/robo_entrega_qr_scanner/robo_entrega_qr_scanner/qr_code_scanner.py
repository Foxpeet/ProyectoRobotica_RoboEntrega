import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from pyzbar import pyzbar

class QRCodeScanner(Node):

    def __init__(self):
        super().__init__('qr_code_scanner')
        self.publisher_ = self.create_publisher(String, '/detected_person', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit(1)

        self.code_to_name = {
            "ABC123": "Jordan",
            "DEF456": "Alex",
            "GHI789": "Diego",
            "JKL012": "Endika",
            "MNO345": "Jaime",
            "PQR678": "Hao"
        }

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame")
            return

        decoded_objects = pyzbar.decode(frame)
        for obj in decoded_objects:
            code_data = obj.data.decode('utf-8')
            self.get_logger().info(f"Detected data: {code_data}")
            try:
                name_part, code_part = code_data.split(' - ')
                name = self.code_to_name.get(code_part, "Unknown")
                if name == name_part:
                    msg = String()
                    msg.data = name
                    self.publisher_.publish(msg)
            except ValueError:
                self.get_logger().warn("Invalid QR format detected.")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
