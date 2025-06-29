import rclpy
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import enum

class AvoidancePhase(enum.Enum):
    INACTIVE = -1
    APPROACHING = 0
    DRIVING_PAST = 1

class ObstacleAvoidanceManager:
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        self.phase = AvoidancePhase.INACTIVE
        self.obstacle_counter = 0
        self.drive_past_timer = None

    def start(self):
        self.logger.info("ObstacleAvoidanceManager: Baustellenmodus gestartet.")
        self.obstacle_counter = 0
        self.phase = AvoidancePhase.APPROACHING

    def execute(self):
        if self.phase == AvoidancePhase.INACTIVE:
            return
        elif self.phase == AvoidancePhase.APPROACHING:
            self._approach()
        elif self.phase == AvoidancePhase.DRIVING_PAST:
            self._drive_past()

    def is_finished(self):
        return self.phase == AvoidancePhase.INACTIVE

    def _approach(self):
        self.logger.info(f"Baustelle: Nähere mich Hindernis #{self.obstacle_counter + 1}...")
        
        if not self.node.lidar_ranges:
            self.node.stop_robot()
            return

        front_ranges = self.node.lidar_ranges[345:360] + self.node.lidar_ranges[0:16]
        front_dist = min([r for r in front_ranges if np.isfinite(r)]) if any(np.isfinite(r) for r in front_ranges) else float('inf')
        trigger_dist = 0.45

        if front_dist < trigger_dist:
            self.logger.info(f"Hindernis #{self.obstacle_counter + 1} getriggert! Starte Ausweich-Spurwechsel.")
            
            
            if self.obstacle_counter == 0: # 1. Hindernis (links) -> nach rechts ausweichen
                self.node.follow_white_line = True
                self.logger.info("Ausweichmanöver 1: Wechsle auf die RECHTE (weiße) Linie.")
            elif self.obstacle_counter == 1: # 2. Hindernis (rechts) -> nach links ausweichen
                self.node.follow_white_line = False
                self.logger.info("Ausweichmanöver 2: Wechsle auf die LINKE (gelbe) Linie.")
            elif self.obstacle_counter == 2: # 3. Hindernis (links) -> nach rechts ausweichen
                self.node.follow_white_line = True
                self.logger.info("Ausweichmanöver 3: Wechsle auf die RECHTE (weiße) Linie.")
            
            self.phase = AvoidancePhase.DRIVING_PAST
            self.drive_past_timer = self.node.get_clock().now()
        else:
            # Solange kein Hindernis nah ist, normal der gelben Linie folgen.
            self.node.follow_white_line = False
            self.node.execute_line_following()

    def _drive_past(self):
        drive_duration = 2.5 # timer zum austesten
        self.logger.info(f"Manöver Phase: Fahre an Hindernis #{self.obstacle_counter + 1} vorbei...")
        
        # Folge der neu gewählten Linie für die festgelegte Dauer
        self.node.execute_line_following()

        if (self.node.get_clock().now() - self.drive_past_timer).nanoseconds > (drive_duration * 1e9):
            self.obstacle_counter += 1
            
            if self.obstacle_counter >= 3:
                self.logger.info("Alle Hindernisse passiert. Baustelle beendet.")
                # Nach dem letzten Hindernis zurück zur gelben Linie
                self.node.follow_white_line = False
                self.phase = AvoidancePhase.INACTIVE
            else:
                self.logger.info(f"Hindernis passiert. Kehre zur Lauerstellung zurück.")
                self.phase = AvoidancePhase.APPROACHING