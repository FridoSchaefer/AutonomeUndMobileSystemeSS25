import rclpy
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import enum

class AvoidancePhase(enum.Enum):
    INACTIVE = -1
    APPROACHING_H2 = 0 # Folgt weißer Linie, sucht H2
    EVADING_H2_TURN1 = 1
    EVADING_H2_DRIVE = 2
    EVADING_H2_TURN2 = 3
    EVADING_H2_SEARCH = 4
    APPROACHING_H3 = 5 # Folgt gelber Linie, sucht H3
    EVADING_H3_TURN1 = 6
    EVADING_H3_DRIVE = 7
    EVADING_H3_RETURN2 = 8
    EVADING_H3_SEARCH = 9
    EXITING_ON_WHITE_LINE = 10 # Baustelle ist passiert, folgt weißer Linie
    FINISHED = 11

class ObstacleAvoidanceManager:
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        self.phase = AvoidancePhase.INACTIVE
        self.phase_timer = None

    def start(self):
        self.logger.info("ObstacleAvoidanceManager: Start. Gehe direkt zur Annäherung an H2.")
        self.phase = AvoidancePhase.APPROACHING_H2
    
    def execute(self):
        # Phasen-Weiche
        if self.phase == AvoidancePhase.APPROACHING_H2: self._handle_h2_approach()
        elif self.phase == AvoidancePhase.EVADING_H2_TURN1: self._execute_turn(2.0, 0.4, AvoidancePhase.EVADING_H2_DRIVE)
        elif self.phase == AvoidancePhase.EVADING_H2_DRIVE: self._execute_drive(7.0, AvoidancePhase.EVADING_H2_TURN2)
        elif self.phase == AvoidancePhase.EVADING_H2_TURN2: self._execute_turn(2.0, -0.4, AvoidancePhase.EVADING_H2_SEARCH)
        elif self.phase == AvoidancePhase.EVADING_H2_SEARCH: self._execute_search(is_white_target=False, next_phase=AvoidancePhase.APPROACHING_H3)
        
        elif self.phase == AvoidancePhase.APPROACHING_H3: self._handle_h3_approach()
        elif self.phase == AvoidancePhase.EVADING_H3_TURN1: self._execute_turn(2.0, -0.4, AvoidancePhase.EVADING_H3_DRIVE)
        elif self.phase == AvoidancePhase.EVADING_H3_DRIVE: self._execute_drive(6.5, AvoidancePhase.EVADING_H3_RETURN2)
        elif self.phase == AvoidancePhase.EVADING_H3_RETURN2: self._execute_turn(2.0, 0.4, AvoidancePhase.EVADING_H3_SEARCH)
        elif self.phase == AvoidancePhase.EVADING_H3_SEARCH: self._execute_search(is_white_target=True, next_phase=AvoidancePhase.FINISHED)

    def is_finished(self):
        # Dieser Manager ist nie "fertig", er bleibt im letzten Zustand, bis ein neues Schild kommt.
        return self.phase == AvoidancePhase.FINISHED

    def _get_front_distance(self):
        if not self.node.lidar_ranges: return float('inf')
        front_sector = self.node.lidar_ranges[355:360] + self.node.lidar_ranges[0:6]
        return min([r for r in front_sector if r > 0.1]) if any(r > 0.1 for r in front_sector) else float('inf')

    ### Generische Choreografie-Bausteine ###
    def _execute_turn(self, duration, angular_velocity, next_phase):
        if self.phase_timer is None: self.phase_timer = self.node.get_clock().now()
        direction = "LINKS" if angular_velocity > 0 else "RECHTS"
        self.logger.info(f"Manöver: Drehe nach {direction}...")
        if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (duration * 1e9):
            self.phase = next_phase
            self.phase_timer = self.node.get_clock().now()
        else:
            self.node.publisher_.publish(Twist(angular=Vector3(z=angular_velocity)))

    def _execute_drive(self, duration, next_phase):
        if self.phase_timer is None: self.phase_timer = self.node.get_clock().now()
        self.logger.info("Manöver: Fahre geradeaus...")
        if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (duration * 1e9):
            self.phase = next_phase
            self.phase_timer = self.node.get_clock().now()
        else:
            self.node.publisher_.publish(Twist(linear=Vector3(x=0.05)))

    def _execute_search(self, is_white_target, next_phase):
        target_line_name = "weißer" if is_white_target else "gelber"
        self.logger.info(f"Manöver: Suche nach {target_line_name} Linie...")
        
        white_found = self.node.last_line_msg[0] != -1
        yellow_found = self.node.last_line_msg[1] != -1
        target_line_found = (is_white_target and white_found) or (not is_white_target and yellow_found)

        if target_line_found:
            self.logger.info("Linie gefunden! Manöver beendet.")
            self.phase = next_phase
            self.phase_timer = None
        else:
            self.node.publisher_.publish(Twist(linear=Vector3(x=0.03)))

    ### Lauer-Funktionen ###
    def _handle_h2_approach(self):
        self.logger.info("Warte auf Hindernis 2 (auf weißer Linie)...")
        self.node.follow_white_line = True
        if self._get_front_distance() < 0.35:
            self.logger.info("Hindernis #2 getriggert! Starte Manöver zu gelber Linie.")
            self.phase = AvoidancePhase.EVADING_H2_TURN1
        else:
            self.node.execute_line_following()

    def _handle_h3_approach(self):
        self.logger.info("Warte auf Hindernis 3 (auf gelber Linie)...")
        self.node.follow_white_line = False
        if self._get_front_distance() < 0.3:
            self.logger.info("Hindernis #3 getriggert! Starte Manöver zu weißer Linie.")
            self.phase = AvoidancePhase.EVADING_H3_TURN1
        else:
            self.node.execute_line_following()