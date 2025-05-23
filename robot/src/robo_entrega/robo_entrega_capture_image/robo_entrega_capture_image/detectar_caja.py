import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from time import time


class Ros2OpenCVImageConverter(Node):   

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.publisher_ = self.create_publisher(Image, '/deteccion/detecta_caja', 10)

        self.box_count = 0  
        self.last_box_time = 0 
        self.box_detected_last_frame = False  
        self.detection_timeout = 10

        self.cajas_previas = []
        self.distancia_umbral = 250

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
            if area > 700 and relacionDeAspecto > 1.5:
                current_time = time()

                caja_es_nueva = True
                for (x_prev, y_prev, w_prev, h_prev) in self.cajas_previas:
                        distancia = ((x - x_prev)**2 + (y - y_prev)**2)**0.5
                        if distancia < self.distancia_umbral:
                            caja_es_nueva = False
                            break

                if (not self.box_detected_last_frame or (current_time - self.last_box_time > self.detection_timeout)) and caja_es_nueva:
                    self.box_count += 1
                    self.get_logger().info(f'Nueva caja detectada. Total: {self.box_count}')

                self.cajas_previas = []
                self.cajas_previas.append((x, y, w, h))
                self.last_box_time = current_time
                self.box_detected_last_frame = True

                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, f"Caja {self.box_count}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
            elif 10 < area < 10.2:
                if 0.9 < relacionDeAspecto < 1.1:
                    self.get_logger().info(f'Mancha detectada - Área: {area}')
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.putText(cv_image, "Mancha", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    

        if not any(cv2.contourArea(c) > 700 and float(cv2.boundingRect(c)[2]) / cv2.boundingRect(c)[3] > 1.5 for c in contornos):
            self.box_detected_last_frame = False

        cv2.imshow("Imagen capturada por el robot", cv_image)
        #cv2.imshow("Imagen filtrada por color", res)
                
        cv2.waitKey(1)

        if time() - self.last_box_time > self.detection_timeout:
            self.cajas_previas = []

        img = self.bridge_object.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(img)

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