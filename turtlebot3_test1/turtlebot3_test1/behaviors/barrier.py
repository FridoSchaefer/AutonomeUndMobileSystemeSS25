import rclpy
from geometry_msgs.msg import Twist
import numpy as np
import enum

class BarrierPhase(enum.Enum):
    INACTIVE = 0
    APPROACHING = 1           # Fährt Linie und sucht nach der Schranke
    WAITING_FOR_CLEARANCE = 2 # Steht vor der Schranke und wartet
    FINISHED = 3

class BarrierManager:
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        self.phase = BarrierPhase.INACTIVE

    def start(self):
        """Startet den Schranken-Annäherungsmodus."""
        self.logger.info("BarrierManager: Starte Annäherung an Schranke.")
        self.phase = BarrierPhase.APPROACHING

    def execute(self):
        """Führt den aktuellen Schritt des Schranken-Manövers aus."""
        if self.phase == BarrierPhase.APPROACHING:
            self._approach_barrier()
        elif self.phase == BarrierPhase.WAITING_FOR_CLEARANCE:
            self._check_barrier_status()

    def is_finished(self):
        return self.phase == BarrierPhase.FINISHED

    def _approach_barrier(self):
        """Fährt normal Linie und prüft, ob die Schranke erreicht ist."""
        if not self.node.lidar_ranges:
            self.node.stop_robot()
            return
            
        # Prüfe den vorderen Lidar-Bereich als Trigger
        front_sector = self.node.lidar_ranges[350:360] + self.node.lidar_ranges[0:11]
        min_front_dist = min([r for r in front_sector if r > 0.1]) if any(r > 0.1 for r in front_sector) else float('inf')
        trigger_dist = 0.4 # 40cm

        # Wenn die Schranke erreicht ist, wechsle die Phase
        if min_front_dist < trigger_dist:
            self.logger.info(f"Schranke bei {min_front_dist:.2f}m erreicht. Stoppe und warte.")
            self.phase = BarrierPhase.WAITING_FOR_CLEARANCE
            self.node.stop_robot()
        else:
            # Solange die Schranke nicht erreicht ist, normal weiterfahren
            self.node.execute_line_following()

    def _check_barrier_status(self):
        """Steht vor der Schranke und prüft, wann sie sich öffnet."""
        if not self.node.lidar_ranges:
            self.node.stop_robot()
            return

        front_sector = self.node.lidar_ranges[350:360] + self.node.lidar_ranges[0:11]
        distance_threshold = 0.5
        width_threshold = 5

        close_points_count = sum(1 for d in front_sector if 0.1 < d < distance_threshold)

        self.logger.info(f"Schranken-Check: {close_points_count} nahe Punkte (Schwelle > {width_threshold})")

        if close_points_count > width_threshold:
            self.logger.info("Schranke ist GESCHLOSSEN. Warte...")
            self.node.stop_robot()
        else:
            self.logger.info("===> Schranke ist OFFEN! Manöver beendet.")
            self.phase = BarrierPhase.FINISHED