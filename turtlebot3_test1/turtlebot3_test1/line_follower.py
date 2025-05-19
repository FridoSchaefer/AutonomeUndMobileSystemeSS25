# VERSION 2025
# Created based on line detection algroithm initialy created by Torsten Wieck for Distlab 2023
# Updated and optimized by Justin Schnurbusch <justin.schnurbusch@hochschule-stralsund.de>

from geometry_msgs.msg import Twist, Vector3
import rclpy
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String
import math
import py_trees
from py_trees.common import Status
from py_trees.blackboard import Blackboard


class LineFollower(py_trees.behaviour.Behaviour):
    def __init__(self, name="LineFollower"):
        super().__init__(name)
        self.angular_speed = 0.0
        self.max_linear_speed = 0.05
        self.linear_speed = self.max_linear_speed
        self.deviation = 0.0
        self.last_deviations = []
        self.node = None
        self.publisher = None
        self.subscription = None
        self.blackboard = Blackboard()
        self.no_input_counter = 0
        self.stop_to_backup = False


    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.subscription = self.node.create_subscription(Int64MultiArray, "/infra_line_detection", self.listener_callback, 10)
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)

        self.traffic_light_subscription = self.node.create_subscription(String, "/move_command", self.traffic_light_callback,10)


    def initialise(self):
        self.angular_speed = 0.0
        self.linear_speed = self.max_linear_speed
        self.deviation = 0.0
        self.last_deviations = []


    def update(self):
        return Status.RUNNING


    """check if new deviation is valid before setting it"""
    def set_deviation(self, deviation):
        min_same_deviations = 7
        # if difference between current and last value is smaller than 15, everything is fine
        if abs(self.deviation - deviation) <= 15:
            self.deviation = deviation
            self.last_deviations = []
            return

        # if not, fill list to check if there is a constant change
        if len(self.last_deviations) <= min_same_deviations:
            self.last_deviations.append(deviation)
            return

        # if the numbers in list are too different, remove first element and go on
        if abs(max(self.last_deviations) - min(self.last_deviations)) > 30:
            self.last_deviations.pop(0)
            self.last_deviations.append(deviation)
            return

        # if numbers in list are all close, take the new deviation
        self.deviation = deviation
        self.last_deviations = []


    """calculate the new deviation"""
    def set_angular_speed(self, current_position, optimal_position, line_color):
        # calculate the deviation
        deviation = optimal_position - current_position
        self.feedback_message = (f"custom follower {line_color} - dev {deviation} opt {optimal_position}")
        if abs(deviation) < 150:
            self.set_deviation(deviation)

        direction_adjustment = 1
        if self.deviation < 0:
            direction_adjustment = -1

        # set max values
        max_deviation = 40
        #print ("#########################################", self.yellow_counter)

        max_turning_speed = 1.2

        power = 2

        self.angular_speed = direction_adjustment * min(
            1.0,
            (max_turning_speed / math.pow(max_deviation, power))
            * math.pow(self.deviation, power)
        )


    """Diese Funktion wertet die Rückgabewerte des Line-Detector-Nodes aus und passt die Fahrbahn des Roboters entsprechend an. Der Code dieser FUnktion
    wurde zu Beginn des Projektes vorgegeben."""
    def listener_callback(self, data):
        cmd_vel_data = Twist()

        # check if follow_line is deaktivated or emergency stop is set
        if not self.blackboard.get("follow_line") or self.blackboard.get("em_stop"):
            cmd_vel_data.linear.x=0.0
            cmd_vel_data.angular.z = 0.0
            self.publisher.publish(cmd_vel_data)
            self.feedback_message = ""
            return

        self.blackboard.set("yellow_counter", 0)

        if not self.stop_to_backup:
            # check which line to follow
            self.yellow_counter = self.blackboard.get("yellow_counter") # number of ticks to ignore white line

            # follow yellow
            if not self.blackboard.get("follow_line_white") or self.yellow_counter > 0:
                if self.yellow_counter > 0:
                    self.yellow_counter -= 1 # count down ticks till left turn to dynamically follow lines again after that
                    self.blackboard.set("yellow_counter", self.yellow_counter)
                yellow_x = data.data[1]
                yellow_optimum = 30

                if yellow_x != -1:
                    self.set_angular_speed(yellow_x, yellow_optimum, "yellow")
                    self.linear_speed = self.max_linear_speed
                    self.no_input_counter = 0

            # follow white
            else:
                white_x = data.data[0]
                white_optimum = 285 # 257

                if white_x != -1:
                    self.set_angular_speed(white_x, white_optimum, "white")
                    self.linear_speed = self.max_linear_speed
                    self.no_input_counter = 0

        cmd_vel_data.linear = Vector3()
        cmd_vel_data.linear.x = self.linear_speed
        cmd_vel_data.angular = Vector3()
        cmd_vel_data.angular.z = self.angular_speed
        self.publisher.publish(cmd_vel_data)