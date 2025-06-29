import rclpy
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import enum

class TunnelPhase(enum.Enum):
    INACTIVE = 0
    DRIVING_BLIND_IN = 1 # Kurzes gerades Einfahren
    WALL_FOLLOWING = 2   # Hauptnavigation
    AVOIDING_OBSTACLE = 3# Temporäres Ausweichmanöver
    FINISHED = 4

class AvoidanceSubPhase(enum.Enum):
    # Unter-Phasen für das Ausweichmanöver
    TURNING_AWAY = 0
    DRIVING_PAST = 1
    TURNING_BACK = 2
    REACQUIRING_WALL = 3

class TunnelManager:
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        
        self.phase = TunnelPhase.INACTIVE
        self.avoidance_sub_phase = None
        self.phase_timer = None
        
        # P-Regler-Verstärkung für das Wall Following
        self.wall_follow_p_gain = 1.5
        self.target_wall_distance = 0.2 # 20cm Abstand zur rechten Wand halten

    def start(self):
        """Startet die Tunnelnavigation."""
        self.logger.info("TunnelManager: Starte Navigation.")
        self.phase = TunnelPhase.DRIVING_BLIND_IN
        self.phase_timer = self.node.get_clock().now()

    def execute(self):
        """Führt den aktuellen Schritt der Tunnelnavigation aus."""
        if self.phase == TunnelPhase.DRIVING_BLIND_IN:
            self._drive_blind_in()
        elif self.phase == TunnelPhase.WALL_FOLLOWING:
            self._wall_following()
        elif self.phase == TunnelPhase.AVOIDING_OBSTACLE:
            self._avoid_obstacle()

    def is_finished(self):
        return self.phase == TunnelPhase.FINISHED

    # --- Private Methoden für jede Haupt-Phase ---

    def _drive_blind_in(self):
        """Fährt kurz geradeaus, um sicher in den Tunnel zu gelangen."""
        drive_duration = 2.0 # 2 Sekunden
        self.logger.info("Tunnel Phase: Blindes Einfahren...")
        
        if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (drive_duration * 1e9):
            self.logger.info("Blindes Einfahren beendet. Starte Wall Following.")
            self.phase = TunnelPhase.WALL_FOLLOWING
        else:
            # Langsam geradeaus fahren
            self.node.publisher_.publish(Twist(linear=Vector3(x=0.04)))

    def _wall_following(self):
        """Hauptlogik: Folgt der rechten Wand mit konstantem Abstand."""
        if not self.node.lidar_ranges:
            self.node.stop_robot(); return

        # Lidar-Sektoren für die Steuerung
        right_dist = self.node.lidar_ranges[270] # Strahl genau rechts (90 Grad)
        front_dist = self.node.lidar_ranges[0]   # Strahl genau vorne
        
        # --- Trigger für Tunnelausgang ---
        # Wenn rechts und vorne plötzlich frei ist, sind wir draußen.
        if right_dist > 2.0 and front_dist > 2.0:
            self.logger.info("===> Tunnelausgang erkannt! Manager beendet.")
            self.phase = TunnelPhase.FINISHED
            return

        # --- Trigger für Hindernisumfahrung ---
        if front_dist < 0.35:
            self.logger.info("Hindernis im Tunnel erkannt! Starte Ausweichmanöver.")
            self.phase = TunnelPhase.AVOIDING_OBSTACLE
            self.avoidance_sub_phase = AvoidanceSubPhase.TURNING_AWAY
            return

        # --- P-Regler für Wall Following ---
        error = self.target_wall_distance - right_dist
        angular_z = self.wall_follow_p_gain * error
        
        # Begrenze die Winkelgeschwindigkeit
        max_angular = 0.5
        angular_z = max(-max_angular, min(max_angular, angular_z))
        
        # Langsam und stetig vorwärts
        linear_x = 0.04
        
        self.logger.info(f"Wall Following: Abstand Rechts={right_dist:.2f}m, Fehler={error:.2f}, AngularZ={angular_z:.2f}")
        self.node.publisher_.publish(Twist(linear=Vector3(x=linear_x), angular=Vector3(z=angular_z)))
        
    def _avoid_obstacle(self):
        """Führt eine feste Sequenz zum Umfahren eines runden Hindernisses aus."""
        
        # Phase 1: Nach links drehen (weg von der rechten Wand)
        if self.avoidance_sub_phase == AvoidanceSubPhase.TURNING_AWAY:
            turn_duration = 2.0 # Zeit für eine 90-Grad-Drehung
            if self.phase_timer is None: self.phase_timer = self.node.get_clock().now()
            
            self.logger.info("Ausweichmanöver: Drehe nach links...")
            if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (turn_duration * 1e9):
                self.avoidance_sub_phase = AvoidanceSubPhase.DRIVING_PAST
                self.phase_timer = self.node.get_clock().now()
            else:
                self.node.publisher_.publish(Twist(angular=Vector3(z=0.5))) # Links drehen

        # Phase 2: Geradeaus am Hindernis vorbei fahren
        elif self.avoidance_sub_phase == AvoidanceSubPhase.DRIVING_PAST:
            drive_duration = 3.0 # Zeit, um am Hindernis vorbeizukommen
            self.logger.info("Ausweichmanöver: Fahre am Hindernis vorbei...")
            if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (drive_duration * 1e9):
                self.avoidance_sub_phase = AvoidanceSubPhase.TURNING_BACK
                self.phase_timer = self.node.get_clock().now()
            else:
                self.node.publisher_.publish(Twist(linear=Vector3(x=0.04)))

        # Phase 3: Zurück zur Wand drehen
        elif self.avoidance_sub_phase == AvoidanceSubPhase.TURNING_BACK:
            turn_duration = 2.0
            self.logger.info("Ausweichmanöver: Drehe zurück zur Wand...")
            if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (turn_duration * 1e9):
                self.avoidance_sub_phase = AvoidanceSubPhase.REACQUIRING_WALL
                self.phase_timer = None # Timer zurücksetzen
            else:
                self.node.publisher_.publish(Twist(angular=Vector3(z=-0.5))) # Rechts drehen

        # Phase 4: Zurück zum Wall Following Modus
        elif self.avoidance_sub_phase == AvoidanceSubPhase.REACQUIRING_WALL:
            self.logger.info("Ausweichmanöver beendet. Kehre zum Wall Following zurück.")
            self.phase = TunnelPhase.WALL_FOLLOWING
            self.avoidance_sub_phase = None