import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int64MultiArray, String, Float32
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import enum

# Importiere die Manager-Klassen
from .behaviors.obstacle_avoidance import ObstacleAvoidanceManager, AvoidancePhase
from .behaviors.parking import ParkingManager
from .behaviors.barrier import BarrierManager
from .behaviors.tunnel import TunnelManager

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
        self.brightness_sub = self.create_subscription(Float32, "/environment/brightness", self.brightness_callback, 10)
        
        # Manager initialisieren
        self.obstacle_manager = ObstacleAvoidanceManager(self, self.get_logger())
        self.parking_manager = ParkingManager(self, self.get_logger())
        self.barrier_manager = BarrierManager(self, self.get_logger())
        self.tunnel_manager = TunnelManager(self, self.get_logger())

        # Zustands-Logik und Regelungs-Variablen
        self.robot_state = RobotState.WAITING_FOR_GREEN_LIGHT
        self.get_logger().info(f"INIT: Starte im Zustand: {self.robot_state.name}")
        self.follow_white_line = False
        self.is_at_intersection = False
        
        self.lidar_ranges = []
        self.current_deviation = 0.0
        self.last_line_data_received = self.get_clock().now()
        self.last_line_msg = [-1, -1]
        self.current_brightness = 128.0

        self.control_timer = self.create_timer(0.1, self.control_loop)

    def traffic_light_callback(self, msg: String):
        if self.robot_state == RobotState.WAITING_FOR_GREEN_LIGHT and msg.data == "green_light":
            self.robot_state = RobotState.LINE_FOLLOWING
            self.get_logger().info(f"Zustandswechsel zu: {self.robot_state.name}")
     
    def line_callback(self, msg: Int64MultiArray):
        self.last_line_data_received = self.get_clock().now()
        self.last_line_msg = msg.data
        white_x, yellow_x = msg.data
        target_position, current_position, line_detected = 0, 0, False
            
        if self.follow_white_line:
            if white_x != -1:
                target_position, current_position, line_detected = self.get_parameter('white_line_optimum').value, white_x, True
        else:
            if yellow_x != -1:
                target_position, current_position, line_detected = self.get_parameter('yellow_line_optimum').value, yellow_x, True
        
        if line_detected: self.current_deviation = target_position - current_position
        else: self.current_deviation = 0.0

    def lidar_callback(self, msg: LaserScan):
        self.lidar_ranges = msg.ranges

    def brightness_callback(self, msg: Float32):
        self.current_brightness = msg.data
        brightness_threshold = 50.0
        if self.robot_state == RobotState.APPROACHING_TUNNEL and self.current_brightness < brightness_threshold:
            self.get_logger().info(f"Dunkelheit erkannt! Starte Tunnelnavigation.")
            self.robot_state = RobotState.NAVIGATING_TUNNEL
            self.tunnel_manager.start()

    def sign_callback(self, msg: String):
        sign = msg.data
        self.get_logger().info(f"Schild erkannt: '{sign}'")
        
        # Normale Fahrt oder Kreisel
        if self.robot_state == RobotState.LINE_FOLLOWING:
            if sign == "intersection": self.is_at_intersection = True
            elif self.is_at_intersection and sign in ["right", "left"]:
                self.robot_state = RobotState.ROUNDABOUT_NAVIGATION
                self.follow_white_line = (sign == "right")
                self.is_at_intersection = False
        
        # 1. Park-Schild beendet IMMER den Baustellen-Modus
        if sign == "park" and self.robot_state == RobotState.OBSTACLE_AVOIDANCE:
            self.get_logger().info("Park-Schild nach Baustelle erkannt. Kehre zu normaler Fahrt zurück.")
            self.robot_state = RobotState.LINE_FOLLOWING
            self.follow_white_line = False # Zurück zur gelben Linie für die Parkplatzsuche
        
        # 2. Baustellenschild starten (nur wenn nicht schon in einem anderen Manöver)
        if sign == "construction" and self.robot_state in [RobotState.LINE_FOLLOWING, RobotState.ROUNDABOUT_NAVIGATION]:
            self.get_logger().info("===> Baustelle erkannt! Leite sofortigen Spurwechsel nach rechts ein.")
            self.robot_state = RobotState.OBSTACLE_AVOIDANCE
            self.follow_white_line = True
            self.obstacle_manager = ObstacleAvoidanceManager(self, self.get_logger())
            self.obstacle_manager.start()

        # 3. Park-Schild starten (nur wenn nicht schon geparkt wird)
        if sign == "park" and self.robot_state != RobotState.PARKING:
            self.get_logger().info("===> Park-Schild erkannt! Übergebe Kontrolle an ParkingManager.")
            self.robot_state = RobotState.PARKING
            self.parking_manager = ParkingManager(self, self.get_logger())
            self.parking_manager.start()
        
        # Schranke
        if sign == "stop" and self.robot_state != RobotState.WAITING_FOR_BARRIER:
            self.robot_state = RobotState.WAITING_FOR_BARRIER
            self.barrier_manager = BarrierManager(self, self.get_logger())
            self.barrier_manager.start()
        
        # Tunnel
        if sign == "tunnel" and self.robot_state != RobotState.APPROACHING_TUNNEL:
            self.robot_state = RobotState.APPROACHING_TUNNEL

    def control_loop(self):
        """Haupt-Zustandsweiche, die die Kontrolle delegiert."""
        if self.robot_state == RobotState.WAITING_FOR_GREEN_LIGHT:
            self.stop_robot()
            return
        
        is_in_maneuver = False
        
        if self.robot_state == RobotState.OBSTACLE_AVOIDANCE:
            self.obstacle_manager.execute()
            # Der Manager wird nicht mehr durch is_finished() beendet, sondern durch ein Schild.
            is_in_maneuver = self.is_in_active_maneuver()
        
        elif self.robot_state == RobotState.PARKING:
            self.parking_manager.execute()
            if self.parking_manager.is_finished():
                self.robot_state = RobotState.LINE_FOLLOWING
        
        elif self.robot_state == RobotState.WAITING_FOR_BARRIER:
            self.barrier_manager.execute()
            if self.barrier_manager.is_finished():
                self.robot_state = RobotState.LINE_FOLLOWING
        
        elif self.robot_state == RobotState.NAVIGATING_TUNNEL:
            self.tunnel_manager.execute()
            if self.tunnel_manager.is_finished():
                self.robot_state = RobotState.LINE_FOLLOWING
        
        # Führe die Linienverfolgung aus, wenn kein blockierendes Manöver aktiv ist.
        if not is_in_maneuver and self.robot_state in [
            RobotState.LINE_FOLLOWING, 
            RobotState.ROUNDABOUT_NAVIGATION, 
            RobotState.OBSTACLE_AVOIDANCE,
            RobotState.APPROACHING_TUNNEL
        ]:
            self.execute_line_following()

    def is_in_active_maneuver(self):
        """Prüft, ob ein Manager gerade ein aktives, blockierendes Manöver ausführt."""
        if self.robot_state == RobotState.OBSTACLE_AVOIDANCE:
            return self.obstacle_manager.phase not in [
                AvoidancePhase.APPROACHING_H2,
                AvoidancePhase.APPROACHING_H3,
                AvoidancePhase.EXITING_ON_WHITE_LINE 
            ]
        return False

    def execute_line_following(self):
        """Führt einen Schritt der Linienverfolgung aus."""
        time_since_last_line = (self.get_clock().now() - self.last_line_data_received).nanoseconds
        if time_since_last_line > 2e9:
            self.get_logger().warn(f"Keine Liniendaten seit {time_since_last_line / 1e9:.2f}s. Stoppe.")
            self.stop_robot()
            return

        p_gain = self.get_parameter('line_p_gain').value
        max_angular = self.get_parameter('max_angular_speed').value
        normalized_deviation = self.current_deviation / 150.0
        
        angular_z = p_gain * normalized_deviation
        
        self.get_logger().info(f"Fahre Linie ({'weiß' if self.follow_white_line else 'gelb'}): dev={self.current_deviation:.1f}, angular_z={angular_z:.2f}")
        
        angular_z = max(-max_angular, min(max_angular, angular_z))
        linear_x = self.get_parameter('max_linear_speed').value
        linear_x = linear_x * (1.0 - 0.8 * abs(normalized_deviation))

        self.publisher_.publish(Twist(linear=Vector3(x=linear_x), angular=Vector3(z=angular_z)))

    def stop_robot(self):
        self.publisher_.publish(Twist())

def main(args=None):
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