# VERSION 2025
# Created based on line detection algroithm initialy created by Torsten Wieck for Distlab 2023
# Updated and optimized by Justin Schnurbusch <justin.schnurbusch@hochschule-stralsund.de>
import cv2
import numpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray
import py_trees
from py_trees.common import Status
from py_trees.blackboard import Blackboard

# algorithm refers to https://gitlab.hochschule-stralsund.de/torsten.wieck/turtlebot3-anleitung/-/blob/main/autonomemobilesysteme_teamprojekt-main/schranke/schranke/line_detector.py?ref_type=heads

"""create an image with canny edge detection"""
def canny_edge_detection(image):
    # Grey ad blur for better edge detection
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_gray, (3, 3), 0)

    # dynamically calculate the thresholds, with the median pixel value
    sigma = 0.7
    md = numpy.median(img_blur)
    lower_threshold = int(max(0, (1.0 - sigma) * md))
    upper_threshold = int(max(0, (1.0 + sigma) * md))

    # apply the canny edge detection
    canny_edges = cv2.Canny(image=img_blur, threshold1=lower_threshold, threshold2=upper_threshold, L2gradient=True)

    return canny_edges


"""create the region of interest"""
def get_roi(image):
    image_height = image.shape[0]

    # draw rectange region of interest
    polygons = numpy.array([[(14, image_height), (320, image_height), (155, 100)]])

    # fill image black
    mask = numpy.zeros_like(image)
    # draw roi in white
    cv2.fillPoly(mask, polygons, 255)

    # apply roi on image
    masked_image = cv2.bitwise_and(image, mask)
    #cv2.imshow("Maske", masked_image)
    return masked_image


"""create the region of interest"""
def h_line_get_roi(image):
    roi_x1 = 130
    roi_x2 = 190
    roi_y1 = 185
    roi_y2 = 220

    # draw rectange region of interest
    polygons = numpy.array([[(roi_x2, roi_y1), (roi_x1, roi_y1), (roi_x1, roi_y2), (roi_x2, roi_y2)]])

    # fill image black
    mask = numpy.zeros_like(image)
    # draw roi in white
    cv2.fillPoly(mask, polygons, [255, 255, 255])

    # apply roi on image
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


"""draw lines on the image"""
def create_line_image(image, lines):
    # create blank image
    line_image = numpy.zeros_like(image)
    # draw every line on the image
    if lines is not None:
        for x1, y1, x2, y2, b, g, r in lines:
            cv2.line(line_image, (x1, y1), (x2, y2), (int(b), int(g), int(r)), 1)

    return line_image


"""take the function parameter and draw a graph at the bottom of the image"""
def make_coordinates(image, line_parameters, b, g, r):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * (3 / 5))
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return numpy.array([x1, y1, x2, y2, b, g, r])


"""
take the endpoints of the detected lines
and put them in a function with the form y=slope*x+intercept
"""
def calc_average_lines(image, lines):
    left_lines = []
    right_lines = []
    detected_lines = []

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = numpy.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        # catch weird slopes of 0.0
        if slope < 0.1 and slope > -0.1:
            continue

        if slope < 0:
            left_lines.append((slope, intercept))
        else:
            right_lines.append((slope, intercept))

    detected_lines = []
    if len(left_lines) > 0:
        left_line_average = numpy.average(left_lines, axis=0)
        detected_lines.append(make_coordinates(image, left_line_average, 0, 255, 255))

    if len(right_lines) > 0:
        right_line_average = numpy.average(right_lines, axis=0)
        detected_lines.append(make_coordinates(image, right_line_average, 255, 255, 255))

    return numpy.array(detected_lines)


"""Mask the left half of the image"""
def mask_left_half(image):
    height, width = image.shape[:2]
    mask = numpy.zeros_like(image)
    mask[:, :width // 2] = 255  # Left half is white (255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def mask_right_half(image):
    height, width = image.shape[:2]
    mask = numpy.zeros_like(image)
    mask[:, width // 2:] = 255  # Right half is white (255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


"""Diese Klasse implementiert das Verhalten des LinienDetectors.
Dazu wird eine Subscription am Topic /camera/image_raw getätigt, das Bild in der Variable self.current_frame zwischengespeichert und bei jedem Tick des Nodes wird das Bild ausgewertet."""
class ImageListener(py_trees.behaviour.Behaviour):
    def __init__(self, name="ImageListener"):
        super(ImageListener, self).__init__(name)
        self.plt_shown = False
        self.calibration_y = 225  # 237 Vertical line on which to detect the lines
        self.last_white_x = None
        self.last_yellow_x = None
        self.error_occurred = False
        self.blackboard = Blackboard()
        self.br = CvBridge()
        self.count_no_white = 0
        self.count_no_yellow = 0
        self.current_frame = []

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.subscription = self.node.create_subscription(Image, "/camera/image_raw", self.listener_callback, 10)
        self.publisher = self.node.create_publisher(Int64MultiArray, "/infra_line_detection", 10)


    """Process the camera image to calculate the appropiate velocities"""
    def listener_callback(self, data):
        try:
            #self.logger.debug("Test Callback")
            # Get Image
            current_frame = self.br.imgmsg_to_cv2(data)
            current_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

            self.current_frame = cv2.resize(current_frame, [320, 240])

        except Exception as e:
            self.blackboard.set("em_stop", True)
            self.feedback_message = "EM_STOP! Exception occured: {}".format(e)
            self.error_occurred = True
            print(f"Error in listener_callback: {e}")


    def update(self):
    # check if follow line is activated
        if(not self.blackboard.get("follow_line")):
            # show only camera image
            cv2.imshow("Camera", self.current_frame)
            cv2.waitKey(1)
            return Status.RUNNING

        if not len(self.current_frame) == 0:
            # Canny edge Detection
            canny_frame = canny_edge_detection(self.current_frame)

            # Get Region of interest
            roi_image = get_roi(canny_frame)
            #cv2.imshow("ROI", roi_image)
            h_line_roi_preview = h_line_get_roi(self.current_frame)

            left_half_masked = mask_left_half(canny_frame)
            right_half_masked = mask_right_half(canny_frame)

            # hough transformation
            lines = cv2.HoughLinesP(roi_image, 1, numpy.pi / 180, 80, numpy.array([]), minLineLength=10, maxLineGap=5,)

            # Draw lines into image if there are any
            def draw_lines(lines, frame):
                if lines is not None:
                    avarage_lines = calc_average_lines(frame, lines)
                    return create_line_image(frame, avarage_lines)
                else:
                    return create_line_image(frame, None)

            lines_only = draw_lines(lines, self.current_frame)

            # Draw found lines into the image
            image_with_lines = cv2.addWeighted(self.current_frame, 0.8, lines_only, 1, 1)

            yellow_optimum = 25 #54
            white_optimum = 285 #257

            # region of interest for yellow only left half
            yellow = self.detect_line(self.calibration_y, self.current_frame, left_half_masked, white_mode=False)
            white = self.detect_line(self.calibration_y, self.current_frame, right_half_masked, white_mode=True)
            self.last_white_x = white
            self.last_yellow_x = yellow
            camera_image = image_with_lines


            cv2.line(camera_image,(0, self.calibration_y),(320, self.calibration_y),(255, 0, 0), 2,)
            # blue horizontal lines on h_line_roi_image
            cv2.line(h_line_roi_preview, (0, 185), (320, 185), (255, 0, 0), 2,)
            cv2.line(h_line_roi_preview, (0, 220), (320, 220), (255, 0, 0), 2,)

            # green optimum lines
            cv2.line(camera_image, (yellow_optimum, 0), (yellow_optimum, 360), (0, 255, 0), 1,)
            cv2.line(camera_image, (white_optimum, 0), (white_optimum, 360), (0, 255, 0), 1,)

            if yellow is not None:
                self.count_no_yellow = 0
                cv2.line(camera_image, (yellow, 0), (yellow, 360), (0, 255, 255), 2,)
                cv2.line(camera_image, (yellow, 100), (yellow_optimum, 100), (0, 0, 255), 2,)

            if white is not None:
                self.count_no_white = 0
                cv2.line(camera_image, (white, 0), (white, 360), (255, 255, 255), 2,)
                cv2.line(camera_image, (white, 100), (white_optimum, 100), (0, 0, 255), 2,)

            msg = Int64MultiArray()

            # replace none with -1 so message can handle it
            # dynamicallly switch between lines
            if white is None:
                white = -1
                self.count_no_white += 1
                if (self.count_no_white == 3):
                    self.blackboard.set("follow_line_white", False)
            if yellow is None:
                yellow = -1
                self.count_no_yellow += 1
                if (self.count_no_yellow == 3):
                    self.blackboard.set("follow_line_white", True)

            publish_array = [int(white), int(yellow)]
            msg.data = publish_array

            print("Sending", msg.data[0], msg.data[1])
            self.publisher.publish(msg)
            cv2.imshow("Camera", camera_image)
            cv2.waitKey(1)

        return Status.RUNNING


    """Take the color and line image to detect the white or yellow line"""
    def detect_line(self, calibration_y, color_image, line_image, white_mode):
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HLS)

        # set hsv values for white/yellow
        # ⚠️ Values for use in the lab, but these are not stable.
        # if white_mode:
        #     h_min = 0
        #     h_max = 255
        #     l_min = 200
        #     l_max = 255
        #     s_min = 0
        #     s_max = 255
        # else:
        #     h_min = 70
        #     h_max = 100
        #     l_min = 60
        #     l_max = 100
        #     s_min = 0
        #     s_max = 40

        # hls for gazebo
        if white_mode:
            h_min = 0
            h_max = 0
            l_min = 95
            l_max = 170
            s_min = 0
            s_max = 0
        else:
            h_min = 10
            h_max = 40
            l_min = 50
            l_max = 100
            s_min = 250
            s_max = 255
        indices = numpy.where(line_image != [0])

        # swap indices to get x,y
        coordinates = zip(indices[1], indices[0])

        # get all edge point on relevant line
        edges_on_horizontal = []

        for coord in coordinates:
            if coord[1] == calibration_y:
                edges_on_horizontal.append(coord)

        # sort list
        edges_on_horizontal = sorted(edges_on_horizontal, key=lambda coord: coord[1], reverse=white_mode)

        found_edges = []
        for edge in edges_on_horizontal:
            # weird y and x swapping again
            (h, l, s) = color_image[edge[1], edge[0]]
            print(edge[0], "::::::", h, " ", l, " ", s)

            if (h >= h_min and h <= h_max and s >= s_min and s <= s_max and l >= l_min and l <= l_max):
                found_edges.append(edge[0])

        # No Edge found
        if len(found_edges) == 0:
            return None

        if white_mode:
            if self.last_white_x is None:
                return found_edges[0]
            # return the closest value to the last measurement
            return max(found_edges)

        else:
            if self.last_yellow_x is None:
                return found_edges[0]
            # return the closest value to the last measurement
            return min(found_edges, key=lambda x: abs(x - self.last_yellow_x))
        

    
