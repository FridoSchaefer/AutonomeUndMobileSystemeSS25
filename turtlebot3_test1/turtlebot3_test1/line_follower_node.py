# line_follower_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int64MultiArray, String
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import enum

class RobotState(enum.Enum):
    WAITING_FOR_GREEN_LIGHT = 0
    LINE_FOLLOWING = 1
    ROUNDABOUT_NAVIGATION = 2
    OBSTACLE_AVOIDANCE = 3

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

        # Zustands-Logik
        self.robot_state = RobotState.WAITING_FOR_GREEN_LIGHT
        self.get_logger().info(f"INIT: Starte im Zustand: {self.robot_state.name}")
        self.follow_white_line = False
        self.is_at_intersection = False
        # Variable, um die rohen Liniendaten zu speichern
        self.last_line_msg = [-1, -1] # Initialisiere mit "keine Linie"
        self.avoidance_phase = 0 # 0=inaktiv, 1=drehen, 2=vorwärts, 3=zurückdrehen, 4=linie suchen
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Regelungs-Variablen
        self.lidar_ranges = []
        self.current_deviation = 0.0
        self.last_line_data_received = self.get_clock().now()
        
        # Baustellen-Logik
        self.obstacle_counter = 0
        self.drive_phase_timer = None

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

        if self.follow_white_line:
            if white_x != -1:
                target_position, current_position, line_detected = self.get_parameter('white_line_optimum').value, white_x, True
        else:
            if yellow_x != -1:
                target_position, current_position, line_detected = self.get_parameter('yellow_line_optimum').value, yellow_x, True
        
        if line_detected:
            self.current_deviation = target_position - current_position
        else:
            # Wenn die verfolgte Linie verloren geht, nicht lenken und langsam weiterfahren
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
        
        if msg.data == "construction" and self.robot_state != RobotState.OBSTACLE_AVOIDANCE:
            self.robot_state = RobotState.OBSTACLE_AVOIDANCE
            self.obstacle_counter = 0
            self.avoidance_phase = 0 # Starte sofort mit der ersten Phase des Manövers
            self.get_logger().info("Baustelle erkannt. Starte Umfahrungsmanöver.")

    def control_loop(self):
        if self.robot_state == RobotState.WAITING_FOR_GREEN_LIGHT:
            self.stop_robot()
            return
        elif self.robot_state == RobotState.LINE_FOLLOWING or self.robot_state == RobotState.ROUNDABOUT_NAVIGATION:
            self.execute_line_following()
        elif self.robot_state == RobotState.OBSTACLE_AVOIDANCE:
            self.execute_obstacle_avoidance()

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

    def execute_obstacle_avoidance(self):
        """
        Im Baustellen-Modus: Fährt normal Linie, bis ein Hindernis nah ist,
        dann wird ein festes Umfahrungsmanöver ausgelöst.
        """
        if not self.lidar_ranges:
            self.stop_robot(); return

        # --- Trigger-Bedingung ---
        front_ranges = self.lidar_ranges[345:360] + self.lidar_ranges[0:16] # 30-Grad-Kegel
        front_dist = min([r for r in front_ranges if np.isfinite(r)]) if any(np.isfinite(r) for r in front_ranges) else float('inf')
        trigger_dist = 0.4 # 40cm

        # Wenn gerade kein Manöver aktiv ist, prüfen wir, ob eins gestartet werden soll.
        if self.avoidance_phase == 0:
            if front_dist < trigger_dist:
                self.get_logger().info(f"Hindernis {self.obstacle_counter + 1} getriggert! Starte Manöver.")
                self.avoidance_phase = 1 # Starte das Ausweichmanöver
            else:
                # Kein Hindernis nah genug -> normal weiter Linie folgen.
                self.execute_line_following()
            return # Wichtig: Funktion hier verlassen

        # --- Phasen-Logik für das aktive Manöver ---
        # Dieser Teil wird nur ausgeführt, wenn avoidance_phase > 0 ist.

        safety_dist_turning = trigger_dist + 0.1 # Etwas mehr Puffer beim Drehen

        # Phase 1: Drehe nach rechts, bis der Weg frei ist
        if self.avoidance_phase == 1:
            self.get_logger().info("Phase 1: Drehe rechts, bis frei...")
            if front_dist > safety_dist_turning:
                self.get_logger().info("Weg ist frei. Wechsle zu Phase 2.")
                self.avoidance_phase = 2
                self.drive_phase_timer = self.get_clock().now()
            else:
                self.publisher_.publish(Twist(angular=Vector3(z=-0.4)))
            return

        # Phase 2: Fahre eine feste Zeit geradeaus
        elif self.avoidance_phase == 2:
            drive_duration = 2.0
            self.get_logger().info("Phase 2: Fahre geradeaus...")
            if (self.get_clock().now() - self.drive_phase_timer).nanoseconds > (drive_duration * 1e9):
                self.get_logger().info("Vorwärtsfahrt beendet. Wechsle zu Phase 3.")
                self.avoidance_phase = 3
                self.drive_phase_timer = self.get_clock().now()
            else:
                self.publisher_.publish(Twist(linear=Vector3(x=0.05)))
            return

        # Phase 3: Drehe zurück nach links
        elif self.avoidance_phase == 3:
            turn_duration = 1.5
            self.get_logger().info("Phase 3: Drehe zurück nach links...")
            if (self.get_clock().now() - self.drive_phase_timer).nanoseconds > (turn_duration * 1e9):
                self.get_logger().info("Rückdrehung beendet. Wechsle zu Phase 4.")
                self.avoidance_phase = 4
            else:
                self.publisher_.publish(Twist(angular=Vector3(z=0.4)))
            return

        # Phase 4: Suche die Linie
        elif self.avoidance_phase == 4:
            self.get_logger().info("Phase 4: Suche nach der Linie...")
            
            ### GEÄNDERT: Korrekte Überprüfung der Liniendaten ###
            # Wir prüfen, ob entweder die weiße oder die gelbe Linie erkannt wurde.
            # self.last_line_msg ist eine Liste wie [white_x, yellow_x]
            line_found = any(val != -1 for val in self.last_line_msg)

            if line_found:
                 self.get_logger().info("Linie wiedergefunden!")
                 self.obstacle_counter += 1
                 if self.obstacle_counter >= 3:
                     self.get_logger().info("Alle 3 Hindernisse umfahren. Baustelle beendet.")
                     self.robot_state = RobotState.LINE_FOLLOWING
                 else:
                     self.get_logger().info(f"Hindernis {self.obstacle_counter} umfahren. Suche nächstes.")
                 
                 # WICHTIG: Manöver beenden und zurück zur normalen Linienverfolgung im Baustellenmodus
                 self.avoidance_phase = 0 
            else:
                 # Langsam vorwärts fahren, bis Linie gefunden wird
                 self.publisher_.publish(Twist(linear=Vector3(x=0.03)))
            return
        
        
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