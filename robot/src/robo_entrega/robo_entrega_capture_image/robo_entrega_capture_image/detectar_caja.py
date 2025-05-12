import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Ros2OpenCVImageConverter(Node):   

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_brown = np.array([10, 100, 20])
        upper_brown = np.array([20, 200, 200])

        lower_dirty = np.array([0, 0, 0])
        upper_dirty = np.array([180, 50, 50])

        mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)
        mask_dirty = cv2.inRange(hsv, lower_dirty, upper_dirty)

        mask_combined = cv2.bitwise_or(mask_brown, mask_dirty)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask_combined)

        img_gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        ret, umbral = cv2.threshold(img_gray, 50, 255, cv2.THRESH_BINARY)

        contornos, jerarquia  = cv2.findContours(umbral, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contorno in contornos:
            area = cv2.contourArea(contorno)
            x, y, w, h = cv2.boundingRect(contorno)
            relacionDeAspecto = float(w)/h
            if area > 700:
                if relacionDeAspecto > 1.5:
                    self.get_logger().info(f'Caja detectada - Area: {area}')
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image, "Caja", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            elif 10 < area < 500:
                if 0.9 < relacionDeAspecto < 1.1:
                    self.get_logger().info(f'Mancha detectada - Área: {area}')
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        cv2.putText(cv_image, "Mancha", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)


        cv2.imshow("Imagen capturada por el robot", cv_image)
        #cv2.imshow("Imagen filtrada por color", res)
                
        cv2.waitKey(1)    

def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = Ros2OpenCVImageConverter()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()