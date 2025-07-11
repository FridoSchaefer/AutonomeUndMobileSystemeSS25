import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from std_msgs.msg import Float32

class SignDetectorNode(Node):
    def __init__(self):
        super().__init__('sign_detector_node')
        self.get_logger().info("Sign Detector Node gestartet.")

        self.debug_mode = True

        package_share_directory = get_package_share_directory('turtlebot3_test1') 
        self.reference_images = {
            "intersection": cv2.imread(f"{package_share_directory}/images/intersection.png"),
            "right": cv2.imread(f"{package_share_directory}/images/right.png"),
            "left": cv2.imread(f"{package_share_directory}/images/left.png"),
            "park": cv2.imread(f"{package_share_directory}/images/park.png"),
            "green_light": cv2.imread(f"{package_share_directory}/images/greenlight.png"),
            "tunnel": cv2.imread(f"{package_share_directory}/images/tunnel.png"),
            "construction": cv2.imread(f"{package_share_directory}/images/construction.png"),
            "stop": cv2.imread(f"{package_share_directory}/images/stop.png"),
        }
        
        for name, img in self.reference_images.items():
            if img is None:
                self.get_logger().error(f"Referenzbild für '{name}' konnte nicht geladen werden!")

        self.bridge = CvBridge()
        self.sift = cv2.SIFT_create()
        self.min_match_count = 10 

        # HSV-Werte für GRÜN
        self.declare_parameter('traffic_light.green.h_min', 45)
        self.declare_parameter('traffic_light.green.h_max', 75)
        self.declare_parameter('traffic_light.green.s_min', 150)
        self.declare_parameter('traffic_light.green.s_max', 255)
        self.declare_parameter('traffic_light.green.v_min', 150)
        self.declare_parameter('traffic_light.green.v_max', 255)
        
        self.declare_parameter('traffic_light.min_area', 100)
        
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.sign_pub = self.create_publisher(String, "/detected_sign", 10)
        self.traffic_light_pub = self.create_publisher(String, "/detected_traffic_light", 10)
        self.brightness_pub = self.create_publisher(Float32, "/environment/brightness", 10)

        self.last_detection_time = {} 
        self.cooldown_duration = rclpy.duration.Duration(seconds=3.0)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            debug_image = cv_image.copy()

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            average_brightness = np.mean(gray_image)
            self.brightness_pub.publish(Float32(data=average_brightness))

            detected_light = self.detect_traffic_light_blob(cv_image, debug_image)
            detected_sign = self.detect_sign_sift(cv_image, debug_image)
            
            self.publish_with_cooldown(detected_light, self.traffic_light_pub)
            self.publish_with_cooldown(detected_sign, self.sign_pub)
            
            if self.debug_mode:
                cv2.imshow("Sign and Light Detection", debug_image)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Fehler in der Bildverarbeitung: {e}")

    def publish_with_cooldown(self, item_name, publisher):
        if item_name is None:
            return
            
        now = self.get_clock().now()
        last_time = self.last_detection_time.get(item_name)
        
        if last_time is None or (now - last_time) > self.cooldown_duration:
            publisher.publish(String(data=item_name))
            self.last_detection_time[item_name] = now

    def detect_traffic_light_blob(self, image, debug_image):
        """Sucht nach großen grünen Farbflecken im oberen Bildbereich."""
        h, w, _ = image.shape
        start_y, end_y = h // 4, h * 3 // 4
        start_x, end_x = w // 2, w
        roi = image[start_y:end_y, start_x:end_x]

        if self.debug_mode:
            cv2.imshow("1 - ROI (Region of Interest)", roi)
        
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        min_area = self.get_parameter('traffic_light.min_area').value

        # grün erkennung
        g_h_min = self.get_parameter('traffic_light.green.h_min').value
        g_h_max = self.get_parameter('traffic_light.green.h_max').value
        g_s_min = self.get_parameter('traffic_light.green.s_min').value
        g_s_max = self.get_parameter('traffic_light.green.s_max').value
        g_v_min = self.get_parameter('traffic_light.green.v_min').value
        g_v_max = self.get_parameter('traffic_light.green.v_max').value
        
        green_mask = cv2.inRange(hsv_roi, (g_h_min, g_s_min, g_v_min), (g_h_max, g_s_max, g_v_max))
        contours_green, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours_green:
            if cv2.contourArea(cnt) > min_area:
                cv2.drawContours(debug_image, [cnt], -1, (0, 255, 0), 3)
                cv2.putText(debug_image, "GREEN LIGHT", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                return "green_light"

                
        return None

    def detect_sign_sift(self, image, debug_image):
        h, w, _ = image.shape

        start_y = h // 10
        end_y = h * 9 // 10
        start_x = w // 10
        end_x = w * 9 // 10
        roi = image[start_y:end_y, start_x:end_x]
        
        if self.debug_mode:
            cv2.rectangle(debug_image, (start_x, start_y), (end_x, end_y), (0, 255, 255), 2)
        
        kp_roi, des_roi = self.sift.detectAndCompute(roi, None)
        if des_roi is None or len(des_roi) < 2:
            return None

        best_match = None
        max_good_matches = 0

        for sign_name, ref_image in self.reference_images.items():
            if ref_image is None: continue
            
            kp_ref, des_ref = self.sift.detectAndCompute(ref_image, None)

            if des_ref is None or len(des_ref) < 2:
                continue
            
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks=50)
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des_ref, des_roi, k=2)

            good_matches = [m for m, n in matches if m.distance < 0.7 * n.distance]

            if len(good_matches) > self.min_match_count and len(good_matches) > max_good_matches:
                max_good_matches = len(good_matches)
                best_match = sign_name
        
        if best_match:
            cv2.putText(debug_image, best_match, (w//2, h//4 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        return best_match

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()