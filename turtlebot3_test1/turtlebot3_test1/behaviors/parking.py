import rclpy
from geometry_msgs.msg import Twist, Vector3
import enum

class ParkingPhase(enum.Enum):
    IDLE = 0
    APPROACHING_SPOT = 1      # Folgt der gelben Linie, bis sie verloren geht
    STOP_AND_SCAN = 2         # Steht still und scannt nach freien Plätzen
    ALIGN_WITH_SPOT = 3       # Dreht sich um 90 Grad zum Parkplatz
    DRIVE_IN = 4              # Fährt eine feste Zeit in die Lücke
    WAITING_IN_SPOT = 5       # Wartet 5 Sekunden in der Lücke
    DRIVE_OUT = 6             # Fährt rückwärts aus der Lücke
    REALIGN_ON_TRACK = 7      # Dreht sich zurück zur Fahrbahn (mit 180-Grad-Wende)
    SEARCHING_FOR_LINE = 8    # Kriecht vorwärts, bis die gelbe Linie wieder da ist
    # <<< NEUE PHASE HIER >>>
    POST_PARK_DRIVING = 9     # Fährt 5s auf der gelben Linie zur Stabilisierung
    FINISHED = 10             # Manöver abgeschlossen

class ParkingManager:
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        
        self.phase = ParkingPhase.IDLE
        self.phase_timer = None
        self.target_spot = None # "left" or "right"

    def start(self):
        """Startet den Parkvorgang mit der Annäherungsphase."""
        self.logger.info("ParkingManager: Starte Parkvorgang. Phase: Annäherung.")
        self.phase = ParkingPhase.APPROACHING_SPOT

    def execute(self):
        """Führt den aktuellen Schritt des Parkmanövers aus."""
        if self.phase == ParkingPhase.APPROACHING_SPOT:
            self._approach_spot()
        elif self.phase == ParkingPhase.STOP_AND_SCAN:
            self._scan_for_spot()
        elif self.phase == ParkingPhase.ALIGN_WITH_SPOT:
            self._align_with_spot()
        elif self.phase == ParkingPhase.DRIVE_IN:
            self._drive_in()
        elif self.phase == ParkingPhase.WAITING_IN_SPOT:
            self._wait_in_spot()
        elif self.phase == ParkingPhase.DRIVE_OUT:
            self._drive_out()
        elif self.phase == ParkingPhase.REALIGN_ON_TRACK:
            self._realign_on_track()
        elif self.phase == ParkingPhase.SEARCHING_FOR_LINE:
            self._search_for_line()
        elif self.phase == ParkingPhase.POST_PARK_DRIVING:
            self._post_park_driving()
            
    def is_finished(self):
        """Gibt True zurück, wenn das Manöver abgeschlossen ist."""
        return self.phase == ParkingPhase.FINISHED

    # --- Private Methoden für jede Phase ---

    def _approach_spot(self):
        self.logger.info("Parking Phase: Folge gelber Linie zum Parkbereich...")
        yellow_line_visible = self.node.last_line_msg[1] != -1
        if yellow_line_visible:
            self.node.execute_line_following()
        else:
            self.logger.info("Gelbe Linie verloren. Stoppe und scanne Parkplätze.")
            self.phase = ParkingPhase.STOP_AND_SCAN
            self.node.stop_robot()

    def _scan_for_spot(self):
        self.logger.info("Parking Phase: Scanne nach freiem Parkplatz (Objekte < 50cm)...")
        if not self.node.lidar_ranges:
            self.logger.warn("Keine Lidar-Daten zum Scannen verfügbar.")
            self.node.stop_robot()
            return
            
        right_spot_sector = self.node.lidar_ranges[265:276]
        left_spot_sector = self.node.lidar_ranges[85:96]
        valid_right = [r for r in right_spot_sector if r > 0.1]
        valid_left = [r for r in left_spot_sector if r > 0.1]
        min_dist_right = min(valid_right) if valid_right else float('inf')
        min_dist_left = min(valid_left) if valid_left else float('inf')
        scan_distance = 0.5
        is_right_free = min_dist_right > scan_distance
        is_left_free = min_dist_left > scan_distance
        self.logger.info(f"Scan-Ergebnis: Rechts frei? {is_right_free} (Dist: {min_dist_right:.2f}m), Links frei? {is_left_free} (Dist: {min_dist_left:.2f}m)")

        spot_chosen = False
        if is_right_free:
            self.logger.info("===> Rechter Parkplatz ist frei. Wähle rechts.")
            self.target_spot = "right"
            spot_chosen = True
        elif is_left_free:
            self.logger.info("===> Linker Parkplatz ist frei. Wähle links.")
            self.target_spot = "left"
            spot_chosen = True
        else:
            self.logger.warn("Beide Parkplätze sind belegt! Bleibe stehen und warte.")
            self.node.stop_robot()

        if spot_chosen:
            self.phase = ParkingPhase.ALIGN_WITH_SPOT
            self.phase_timer = self.node.get_clock().now()

    def _align_with_spot(self):
        turn_duration = 3.0
        angular_z = -0.4 if self.target_spot == "right" else 0.4
        self.logger.info(f"Parking Phase: Richte auf {self.target_spot} Parkplatz aus...")
        if self.phase_timer is not None and (self.node.get_clock().now() - self.phase_timer).nanoseconds > (turn_duration * 1e9):
            self.phase = ParkingPhase.DRIVE_IN
            self.phase_timer = self.node.get_clock().now()
            self.node.stop_robot()
        else:
            self.node.publisher_.publish(Twist(angular=Vector3(z=angular_z)))
    
    def _drive_in(self):
        drive_duration = 7.0
        self.logger.info("Parking Phase: Fahre in Parklücke...")
        if self.phase_timer is not None and (self.node.get_clock().now() - self.phase_timer).nanoseconds > (drive_duration * 1e9):
            self.phase = ParkingPhase.WAITING_IN_SPOT
            self.phase_timer = self.node.get_clock().now()
            self.node.stop_robot()
        else:
            self.node.publisher_.publish(Twist(linear=Vector3(x=0.05)))

    def _wait_in_spot(self):
        wait_duration = 4.0
        self.logger.info("Parking Phase: Warte 4 Sekunden in Parklücke...")
        if self.phase_timer is not None and (self.node.get_clock().now() - self.phase_timer).nanoseconds > (wait_duration * 1e9):
            self.phase = ParkingPhase.DRIVE_OUT
            self.phase_timer = self.node.get_clock().now()
        else:
            self.node.stop_robot()

    def _drive_out(self):
        drive_duration = 7.0
        self.logger.info("Parking Phase: Fahre rückwärts aus Parklücke...")
        if self.phase_timer is not None and (self.node.get_clock().now() - self.phase_timer).nanoseconds > (drive_duration * 1e9):
            self.phase = ParkingPhase.REALIGN_ON_TRACK
            self.phase_timer = self.node.get_clock().now()
            self.node.stop_robot()
        else:
            self.node.publisher_.publish(Twist(linear=Vector3(x=-0.05)))

    def _realign_on_track(self):
        turn_duration = 5.5
        angular_z = -0.4 if self.target_spot == "right" else 0.4 
        self.logger.info("Parking Phase: Drehe um 180 Grad zurück zur Strecke...")
        if self.phase_timer is not None and (self.node.get_clock().now() - self.phase_timer).nanoseconds > (turn_duration * 1e9):
            self.phase = ParkingPhase.SEARCHING_FOR_LINE
            self.node.stop_robot()
        else:
            self.node.publisher_.publish(Twist(angular=Vector3(z=angular_z)))
            
    def _search_for_line(self):
        self.logger.info("Parking Phase: Suche nach der gelben Linie...")
        yellow_line_found = self.node.last_line_msg[1] != -1
        
        if yellow_line_found:
            self.logger.info("Gelbe Linie gefunden. Starte 5s Stabilisierungsfahrt.")
            self.phase = ParkingPhase.POST_PARK_DRIVING
            self.phase_timer = self.node.get_clock().now()
        else:
            # Langsam vorwärts "kriechen", bis die Linie gefunden wird
            self.node.publisher_.publish(Twist(linear=Vector3(x=0.02)))
            

    def _post_park_driving(self):
        """Folgt für eine feste Zeit der gelben Linie, um sich zu stabilisieren."""
        self.logger.info("Parking Phase: Stabilisierungsfahrt auf gelber Linie...")
        drive_duration = 13.5 # 13.5 Sekunden auf der gelben Linie fahren

        if self.phase_timer and (self.node.get_clock().now() - self.phase_timer).nanoseconds > (drive_duration * 1e9):
            self.logger.info("Stabilisierungsfahrt beendet. Parkmanöver vollständig abgeschlossen.")
            self.phase = ParkingPhase.FINISHED
            self.node.stop_robot()
        else:
            self.node.execute_line_following()