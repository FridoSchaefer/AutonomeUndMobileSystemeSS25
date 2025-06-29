import rclpy
from geometry_msgs.msg import Twist, Vector3
import enum

class ParkingPhase(enum.Enum):
    IDLE = 0
    STOP_AND_SCAN = 1
    ALIGN_WITH_SPOT = 2
    DRIVE_IN = 3
    WAITING_IN_SPOT = 4
    DRIVE_OUT = 5
    REALIGN_ON_TRACK = 6
    SEARCHING_FOR_LINE = 7
    FINISHED = 8

class ParkingManager:
    def __init__(self, node, logger):
        """
        Initialisiert den ParkingManager.
        - node: Die ROS2-Node-Instanz, um auf Publisher und Timer zugreifen zu können.
        - logger: Der Logger der Haupt-Node für einheitliches Logging.
        """
        self.node = node
        self.logger = logger
        
        self.phase = ParkingPhase.IDLE
        self.phase_timer = None
        self.target_spot = None # "left" or "right"

    def start(self):
        """Startet den Parkvorgang."""
        self.logger.info("ParkingManager: Starte Parkvorgang.")
        self.phase = ParkingPhase.STOP_AND_SCAN
        # Wir müssen den Roboter stoppen, um zu scannen
        self.node.publisher_.publish(Twist())

    def execute(self):
        """
        Führt den aktuellen Schritt des Parkmanövers aus.
        Wird von der Haupt-control_loop aufgerufen.
        """
        if self.phase == ParkingPhase.IDLE:
            return

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
            
    def is_finished(self):
        """Gibt True zurück, wenn das Manöver abgeschlossen ist."""
        return self.phase == ParkingPhase.FINISHED

    # --- Private Methoden für jede Phase ---

    def _scan_for_spot(self):
        self.logger.info("Parking Phase: Scanne nach freiem Parkplatz...")
        # Lidar-Sektoren für den rechten und linken Parkplatz (angenommen, Roboter schaut geradeaus)
        # Rechter Spot: 90 Grad rechts -> 270 im Lidar
        right_spot_ranges = self.node.lidar_ranges[260:281] # Sektor um 270 Grad
        # Linker Spot: 90 Grad links -> 90 im Lidar
        left_spot_ranges = self.node.lidar_ranges[80:101] # Sektor um 90 Grad

        min_right = min([r for r in right_spot_ranges if r > 0.1]) if any(r > 0.1 for r in right_spot_ranges) else 0
        min_left = min([r for r in left_spot_ranges if r > 0.1]) if any(r > 0.1 for r in left_spot_ranges) else 0

        # Annahme: Parkplatz ist frei, wenn die Distanz groß ist (z.B. > 1 Meter)
        # Wir nehmen den ersten freien Platz von rechts.
        if min_right > 1.0:
            self.logger.info("Rechter Parkplatz ist frei!")
            self.target_spot = "right"
            self.phase = ParkingPhase.ALIGN_WITH_SPOT
            self.phase_timer = self.node.get_clock().now()
        elif min_left > 1.0:
            self.logger.info("Linker Parkplatz ist frei!")
            self.target_spot = "left"
            self.phase = ParkingPhase.ALIGN_WITH_SPOT
            self.phase_timer = self.node.get_clock().now()
        else:
            self.logger.warn("Kein freier Parkplatz gefunden. Bleibe stehen.")

    def _align_with_spot(self):
        turn_duration = 2.0 # Zeit für eine 90-Grad-Drehung (anpassen!)
        angular_z = -0.5 if self.target_spot == "right" else 0.5

        self.logger.info(f"Parking Phase: Richte auf {self.target_spot} Parkplatz aus...")
        if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (turn_duration * 1e9):
            self.phase = ParkingPhase.DRIVE_IN
            self.phase_timer = self.node.get_clock().now()
        else:
            self.node.publisher_.publish(Twist(angular=Vector3(z=angular_z)))
    
    def _drive_in(self):
        drive_duration = 3.0 # Zeit für das Einfahren (anpassen!)
        self.logger.info("Parking Phase: Fahre in Parklücke...")
        if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (drive_duration * 1e9):
            self.phase = ParkingPhase.WAITING_IN_SPOT
            self.phase_timer = self.node.get_clock().now()
        else:
            self.node.publisher_.publish(Twist(linear=Vector3(x=0.04)))

    def _wait_in_spot(self):
        wait_duration = 3.0
        self.logger.info("Parking Phase: Warte in Parklücke...")
        if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (wait_duration * 1e9):
            self.phase = ParkingPhase.DRIVE_OUT
            self.phase_timer = self.node.get_clock().now()
        else:
            self.node.publisher_.publish(Twist()) # Stopp

    def _drive_out(self):
        drive_duration = 3.0 # Dieselbe Zeit wie beim Einfahren
        self.logger.info("Parking Phase: Fahre aus Parklücke...")
        if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (drive_duration * 1e9):
            self.phase = ParkingPhase.REALIGN_ON_TRACK
            self.phase_timer = self.node.get_clock().now()
        else:
            self.node.publisher_.publish(Twist(linear=Vector3(x=-0.04))) # Rückwärts

    def _realign_on_track(self):
        turn_duration = 2.0 # Dieselbe Zeit wie beim Ausrichten
        angular_z = 0.5 if self.target_spot == "right" else -0.5 # In die entgegengesetzte Richtung drehen

        self.logger.info("Parking Phase: Richte zurück zur Strecke aus...")
        if (self.node.get_clock().now() - self.phase_timer).nanoseconds > (turn_duration * 1e9):
            self.phase = ParkingPhase.SEARCHING_FOR_LINE
        else:
            self.node.publisher_.publish(Twist(angular=Vector3(z=angular_z)))
            
    def _search_for_line(self):
        self.logger.info("Parking Phase: Suche nach der Linie...")
        line_found = any(val != -1 for val in self.node.last_line_msg)
        if line_found:
            self.logger.info("Linie wiedergefunden! Parkvorgang abgeschlossen.")
            self.phase = ParkingPhase.FINISHED
        else:
            # Langsam vorwärts "kriechen", bis Linie gefunden wird
            self.node.publisher_.publish(Twist(linear=Vector3(x=0.02)))