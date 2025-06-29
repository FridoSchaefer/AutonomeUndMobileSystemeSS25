# line_follower_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int64MultiArray, String
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import enum
from .behaviors.parking import ParkingManager
from .behaviors.obstacle_avoidance import ObstacleAvoidanceManager
from .behaviors.barrier import BarrierManager
from .behaviors.tunnel import TunnelManager
from std_msgs.msg import Float32


class RobotState(enum.Enum):
    WAITING_FOR_GREEN_LIGHT = 0
    LINE_FOLLOWING = 1
    ROUNDABOUT_NAVIGATION = 2
    OBSTACLE_AVOIDANCE = 3
    PARKING = 4
    WAITING_FOR_BARRIER = 5
    APPROACHING_TUNNEL = 6
    NAVIGATING_TUNNEL = 7

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().info("Line Follower Node gestartet.")

        # Parameter
        self.declare_parameter('max_linear_speed', 0.05)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('line_p_gain', 1.0)
        self.declare_parameter('white_line_optimum', 285)
        self.declare_parameter('yellow_line_optimum', 30)

        # Publisher & Subscriber
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.line_sub = self.create_subscription(Int64MultiArray, "/line_detection", self.line_callback, 10)
        self.sign_sub = self.create_subscription(String, "/detected_sign", self.sign_callback, 10)
        self.traffic_light_sub = self.create_subscription(String, "/detected_traffic_light", self.traffic_light_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        #Manager initialisieren
        self.obstacle_manager = ObstacleAvoidanceManager(self, self.get_logger())
        self.parking_manager = ParkingManager(self, self.get_logger())
        self.barrier_manager = BarrierManager(self, self.get_logger())
        self.tunnel_manager = TunnelManager(self, self.get_logger())

        # Zustands-Logik
        self.robot_state = RobotState.WAITING_FOR_GREEN_LIGHT
        self.get_logger().info(f"INIT: Starte im Zustand: {self.robot_state.name}")
        self.follow_white_line = False
        self.is_at_intersection = False

        # Variable, um die rohen Liniendaten zu speichern
        self.last_line_msg = [-1, -1] # Initialisiere mit "keine Linie"
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Regelungs-Variablen
        self.lidar_ranges = []
        self.current_deviation = 0.0
        self.last_line_data_received = self.get_clock().now()
        

    def traffic_light_callback(self, msg: String):
        if self.robot_state == RobotState.WAITING_FOR_GREEN_LIGHT and msg.data == "green_light":
            self.get_logger().info("GRÜNES LICHT ERKANNT! Engines firing up.")
            self.current_deviation = 0.0
            self.last_line_data_received = self.get_clock().now()
            self.robot_state = RobotState.LINE_FOLLOWING
            self.get_logger().info(f"Zustandswechsel zu: {self.robot_state.name}")
     
    def line_callback(self, msg: Int64MultiArray):
        self.last_line_data_received = self.get_clock().now()
        white_x, yellow_x = msg.data
        target_position, current_position, line_detected = 0, 0, False
        self.last_line_msg = msg.data

        if self.robot_state == RobotState.PARKING:
            self.parking_manager.execute()
            
        if self.follow_white_line:
            if white_x != -1:
                target_position, current_position, line_detected = self.get_parameter('white_line_optimum').value, white_x, True
        else:
            if yellow_x != -1:
                target_position, current_position, line_detected = self.get_parameter('yellow_line_optimum').value, yellow_x, True
        
        if line_detected:
            self.current_deviation = target_position - current_position
        else:
            # linie verloren, langsam weiterfahren
            self.current_deviation = 0.0

    def lidar_callback(self, msg: LaserScan):
        self.lidar_ranges = msg.ranges

    def sign_callback(self, msg: String):
        sign = msg.data
        self.get_logger().info(f"Schild erkannt: '{sign}'")
        
        if self.robot_state == RobotState.LINE_FOLLOWING:
            if sign == "intersection":
                self.is_at_intersection = True
            elif self.is_at_intersection and (sign == "right" or sign == "left"):
                self.robot_state = RobotState.ROUNDABOUT_NAVIGATION
                self.follow_white_line = (sign == "right")
                self.is_at_intersection = False
        
        if sign == "construction" and self.robot_state != RobotState.OBSTACLE_AVOIDANCE:
            self.get_logger().info("===> Baustelle erkannt! Übergebe Kontrolle an ObstacleAvoidanceManager.")
            self.robot_state = RobotState.OBSTACLE_AVOIDANCE
            self.obstacle_manager.start()

        if sign == "park" and self.robot_state != RobotState.PARKING:
            self.get_logger().info("===> Park-Schild erkannt! Übergebe Kontrolle an ParkingManager.")
            self.robot_state = RobotState.PARKING
            self.parking_manager.start()

        if sign == "stop" and self.robot_state != RobotState.WAITING_FOR_BARRIER:
            self.get_logger().info("===> Stopp-Schild erkannt! Wechsle in den Schranken-Modus.")
            self.robot_state = RobotState.WAITING_FOR_BARRIER
            self.barrier_manager.start()

        if msg.data == "tunnel" and self.robot_state != RobotState.APPROACHING_TUNNEL:
            self.get_logger().info("===> Tunnel-Schild erkannt. Wechsle in den Annäherungsmodus.")
            self.robot_state = RobotState.APPROACHING_TUNNEL

    
    def brightness_callback(self, msg: Float32):
        """Wird aufgerufen, wenn neue Helligkeitsdaten ankommen."""
        self.current_brightness = msg.data
        
        # Trigger für den Tunneleintritt
        brightness_threshold = 50.0 # anapssen
        if self.robot_state == RobotState.APPROACHING_TUNNEL and self.current_brightness < brightness_threshold:
            # Wir sind gerade dunkel geworden, das ist der Eingang!
            self.get_logger().info(f"Dunkelheit erkannt (Helligkeit: {self.current_brightness:.1f}). Starte Tunnelnavigation.")
            self.robot_state = RobotState.NAVIGATING_TUNNEL
            self.tunnel_manager.start()


    def control_loop(self):
        if self.robot_state == RobotState.WAITING_FOR_GREEN_LIGHT:
            self.stop_robot()
            return
        
        elif self.robot_state == RobotState.LINE_FOLLOWING or self.robot_state == RobotState.ROUNDABOUT_NAVIGATION:
            self.execute_line_following()

        elif self.robot_state == RobotState.OBSTACLE_AVOIDANCE:
            self.obstacle_manager.execute()
            if self.obstacle_manager.is_finished():
                self.get_logger().info("Baustelle beendet. Kehre zu LINE_FOLLOWING zurück.")
                self.robot_state = RobotState.LINE_FOLLOWING

        elif self.robot_state == RobotState.PARKING:
            self.parking_manager.execute()
            if self.parking_manager.is_finished():
                self.get_logger().info("Park-Manager ist fertig. Kehre zu LINE_FOLLOWING zurück.")
                self.robot_state = RobotState.LINE_FOLLOWING
                # Manager für nächsten Einsatz zurücksetzen
                self.parking_manager.phase = self.parking_manager.ParkingPhase.IDLE

        elif self.robot_state == RobotState.WAITING_FOR_BARRIER:
            self.barrier_manager.execute()
            if self.barrier_manager.is_finished():
                self.get_logger().info("Schranke passiert. Kehre zu LINE_FOLLOWING zurück.")
                self.robot_state = RobotState.LINE_FOLLOWING
                # Manager für nächsten Einsatz zurücksetzen
                self.barrier_manager.__init__(self, self.get_logger())

        elif self.robot_state == RobotState.APPROACHING_TUNNEL:
            # Im Annäherungsmodus einfach weiter Linie fahren
            self.execute_line_following()

        elif self.robot_state == RobotState.NAVIGATING_TUNNEL:
            self.tunnel_manager.execute()
            if self.tunnel_manager.is_finished():
                self.get_logger().info("Tunnel-Manager ist fertig. Suche Linie zum Wiedereinstieg.")
                self.robot_state = RobotState.LINE_FOLLOWING
        

    def execute_line_following(self):
        time_since_last_line = (self.get_clock().now() - self.last_line_data_received).nanoseconds
        # Timeout auf 2.0 Sekunden erhöht, um Performance-Schwankungen abzufangen
        if time_since_last_line > 2e9:
            self.get_logger().warn(f"Keine Liniendaten seit {time_since_last_line / 1e9:.2f}s. Stoppe.")
            self.stop_robot()
            return

        p_gain = self.get_parameter('line_p_gain').value
        max_angular = self.get_parameter('max_angular_speed').value
        
        normalized_deviation = self.current_deviation / 150.0
        angular_z = p_gain * normalized_deviation 
        angular_z = max(-max_angular, min(max_angular, angular_z))

        linear_x = self.get_parameter('max_linear_speed').value
        # Wenn die Abweichung groß ist, fahre langsamer, aber halte nicht komplett an
        linear_x = linear_x * (1.0 - 0.8 * abs(normalized_deviation))

        self.publisher_.publish(Twist(linear=Vector3(x=linear_x), angular=Vector3(z=angular_z)))
        
        
    def stop_robot(self):
        """Sendet einen expliziten Stopp-Befehl."""
        cmd_vel_msg = Twist() # Alle Werte sind per Default 0.0
        self.publisher_.publish(cmd_vel_msg)

def main(args=None):
    """
    Die Hauptfunktion, die den Node initialisiert und startet.
    """
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()