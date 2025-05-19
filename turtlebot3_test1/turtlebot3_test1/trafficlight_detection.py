import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        
        # Subscriber und Publisher
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Bool, '/traffic_light/green', 10)
        
        self.bridge = CvBridge()
        self.green_detected = False
        
        # ROS2-Parameter für HSV-Werte (dynamisch anpassbar)
        self.declare_parameter('lower_green', [40, 50, 50])
        self.declare_parameter('upper_green', [90, 255, 255])
        
        # Timer für Status-Updates
        self.timer = self.create_timer(0.1, self.publish_status)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Dynamische Parameterabfrage
            lower_green = np.array(self.get_parameter('lower_green').get_parameter_value().integer_array_value)
            upper_green = np.array(self.get_parameter('upper_green').get_parameter_value().integer_array_value)
            
            # 1. Maskierung
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            # 2. Rauschen reduzieren
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # 3. ROI auf oberen Bildbereich begrenzen
            height, width = mask.shape[:2]
            roi = mask[0:int(height/3), :]
            
            # 4. Konturenanalyse
            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.green_detected = False
            
            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(max_contour)
                
                if area > 500:  # Schwellenwert anpassen
                    # Optional: Kreisform prüfen
                    perimeter = cv2.arcLength(max_contour, True)
                    circularity = 4 * np.pi * area / (perimeter**2) if perimeter > 0 else 0
                    if circularity > 0.5:
                        self.green_detected = True
            
            # Debug-Bild anzeigen
            cv2.imshow("Detection", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Fehler: {str(e)}")

    def publish_status(self):
        msg = Bool()
        msg.data = self.green_detected
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
