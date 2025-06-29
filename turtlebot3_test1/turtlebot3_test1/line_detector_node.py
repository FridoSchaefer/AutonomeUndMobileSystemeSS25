# line_detector_node.py
import cv2
import numpy
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray

# --- Anfang Hilfsfunktionen ---
def canny_edge_detection(image):
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_gray, (3, 3), 0)
    sigma = 0.7
    md = numpy.median(img_blur)
    lower_threshold = int(max(0, (1.0 - sigma) * md))
    upper_threshold = int(max(0, (1.0 + sigma) * md))
    canny_edges = cv2.Canny(image=img_blur, threshold1=lower_threshold, threshold2=upper_threshold, L2gradient=True)
    return canny_edges

def get_roi(image):
    image_height = image.shape[0]
    polygons = numpy.array([[(14, image_height), (320, image_height), (155, 100)]])
    mask = numpy.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def mask_left_half(image):
    height, width = image.shape[:2]
    mask = numpy.zeros_like(image)
    mask[:, :width // 2] = 255
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def mask_right_half(image):
    height, width = image.shape[:2]
    mask = numpy.zeros_like(image)
    mask[:, width // 2:] = 255
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image
# --- Ende Hilfsfunktionen ---


class LineDetectorNode(Node):
    """
    Dieser Node empfängt Kamerabilder, erkennt weiße und gelbe Linien und 
    veröffentlicht deren erkannte horizontale Position (x-Koordinate).
    """
    def __init__(self):
        super().__init__('line_detector_node')
        self.get_logger().info("Line Detector Node gestartet.")
        
        # ROS Parameter deklarieren für einfache Konfiguration
        self.declare_parameter('calibration_y', 225)
        self.declare_parameter('hsv.white.h_min', 0)
        self.declare_parameter('hsv.white.h_max', 0)
        self.declare_parameter('hsv.white.l_min', 95)
        self.declare_parameter('hsv.white.l_max', 170)
        self.declare_parameter('hsv.white.s_min', 0)
        self.declare_parameter('hsv.white.s_max', 0)
        self.declare_parameter('hsv.yellow.h_min', 10)
        self.declare_parameter('hsv.yellow.h_max', 40)
        self.declare_parameter('hsv.yellow.l_min', 50)
        self.declare_parameter('hsv.yellow.l_max', 100)
        self.declare_parameter('hsv.yellow.s_min', 250)
        self.declare_parameter('hsv.yellow.s_max', 255)

        # CvBridge zum Konvertieren von ROS-Bildern
        self.bridge = CvBridge()
        
        # Subscriber für das Kamerabild
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",  # oder /image_raw je nach Setup
            self.image_callback,
            10)
            
        # Publisher für die erkannten Liniendaten
        self.publisher = self.create_publisher(
            Int64MultiArray,
            "/line_detection",
            10)

        self.last_white_x = None
        self.last_yellow_x = None
        self.show_video = True # Zum Debuggen

    def image_callback(self, msg: Image):
        """Wird bei jedem neuen Kamerabild aufgerufen."""
        try:
            # Konvertiere ROS Image Nachricht zu einem OpenCV Bild
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Fehler bei der Bildkonvertierung: {e}")
            return

        # Bildverarbeitung
        canny_frame = canny_edge_detection(frame)
        
        # Maskiere linke und rechte Hälfte für die jeweilige Liniensuche
        left_half_masked = mask_left_half(canny_frame)
        right_half_masked = mask_right_half(canny_frame)
        
        # Lade Kalibrierungshöhe aus Parametern
        calibration_y = self.get_parameter('calibration_y').get_parameter_value().integer_value

        # Erkenne die Linien
        yellow_x = self.detect_line(calibration_y, frame, left_half_masked, white_mode=False)
        white_x = self.detect_line(calibration_y, frame, right_half_masked, white_mode=True)
        
        # Aktualisiere die letzten bekannten Positionen
        if yellow_x is not None: self.last_yellow_x = yellow_x
        if white_x is not None: self.last_white_x = white_x

        # Bereite die Nachricht vor
        # Sende -1, wenn eine Linie nicht erkannt wurde
        publish_array = [
            int(white_x if white_x is not None else -1),
            int(yellow_x if yellow_x is not None else -1)
        ]
        msg_out = Int64MultiArray(data=publish_array)
        self.publisher.publish(msg_out)

        # Visuelles Feedback (optional, kann Performance kosten)
        if self.show_video:
            # Zeichne Linien für Debugging
            cv2.line(frame, (0, calibration_y), (320, calibration_y), (255, 0, 0), 2)
            if yellow_x is not None:
                cv2.circle(frame, (yellow_x, calibration_y), 5, (0, 255, 255), -1)
            if white_x is not None:
                cv2.circle(frame, (white_x, calibration_y), 5, (255, 255, 255), -1)
            
            cv2.imshow("Line Detection", frame)
            cv2.waitKey(1)

    def detect_line(self, calibration_y, color_image, line_image, white_mode):
        """Sucht nach einer Linie einer bestimmten Farbe auf einer bestimmten Höhe."""
        hls_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HLS)

        # Lade HSV-Werte aus Parametern
        if white_mode:
            p_prefix = 'hsv.white.'
        else:
            p_prefix = 'hsv.yellow.'
            
        h_min = self.get_parameter(p_prefix + 'h_min').value
        h_max = self.get_parameter(p_prefix + 'h_max').value
        l_min = self.get_parameter(p_prefix + 'l_min').value
        l_max = self.get_parameter(p_prefix + 'l_max').value
        s_min = self.get_parameter(p_prefix + 's_min').value
        s_max = self.get_parameter(p_prefix + 's_max').value

        # Finde alle Kantenpixel in der Canny-Map
        indices = numpy.where(line_image != [0])
        coordinates = zip(indices[1], indices[0])

        # Finde alle Kantenpixel auf der relevanten horizontalen Linie
        edges_on_horizontal = [coord for coord in coordinates if coord[1] == calibration_y]

        found_edges = []
        for edge in edges_on_horizontal:
            (h, l, s) = hls_image[edge[1], edge[0]]
            if (h_min <= h <= h_max and l_min <= l <= l_max and s_min <= s <= s_max):
                found_edges.append(edge[0])

        if not found_edges:
            return None

        # Gib den passendsten Wert zurück
        if white_mode:
            last_x = self.last_white_x
            # Wähle den rechtesten Punkt (höchster x-Wert)
            best_match = max(found_edges)
        else:
            last_x = self.last_yellow_x
            # Wähle den linkesten Punkt (niedrigster x-Wert)
            best_match = min(found_edges)
        
        # Wenn wir eine frühere Position kennen, nimm den Punkt, der am nächsten dran ist
        if last_x is not None:
             return min(found_edges, key=lambda x: abs(x - last_x))
        else:
             return best_match


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()