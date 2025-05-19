from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from py_trees import common
import rclpy
from sensor_msgs.msg import Image
import cv2
import enum
import numpy as np
import py_trees as pt
from py_trees.blackboard import Blackboard

# algorithm refers to https://gitlab.hochschule-stralsund.de/torsten.wieck/turtlebot3-anleitung/-/blob/main/autonomemobilesysteme_teamprojekt-main/schranke/schranke/traffic_sign_dection.py?ref_type=heads

class SignType(enum.Enum):
    BARRIER = "barrier"
    TRAIN = "train"
    PARK = "park"


class SignDetection(pt.behaviour.Behaviour):
    def __init__(self):
        super(SignDetection, self).__init__("SignDetector")

        # load reference images
        dir_path = get_package_share_directory("parken_schranke")
        self.reference_images = {
            "barrier0": cv2.imread("%s/reference_image/barrier/0.png" % dir_path),
            "barrier1": cv2.imread("%s/reference_image/barrier/1.png" % dir_path),
            "barrier2": cv2.imread("%s/reference_image/barrier/2.png" % dir_path),
            "train0": cv2.imread("%s/reference_image/train/0.png" % dir_path),
            #"train1": cv2.imread("%s/reference_image/train/1.png" % dir_path),
            #"train2": cv2.imread("%s/reference_image/train/2.png" % dir_path),
            #"train3": cv2.imread("%s/reference_image/train/3.png" % dir_path),
            #"park0": cv2.imread("%s/reference_image/park/0.png" % dir_path),
            "park1": cv2.imread("%s/reference_image/park/1.png" % dir_path),
            "park2": cv2.imread("%s/reference_image/park/2.png" % dir_path),
            #"park3": cv2.imread("%s/reference_image/park/3.png" % dir_path),
        }

        self.min_match_count = 5
        self.barrier_closed = False
        self.count = 0
        self.blackboard = Blackboard()
        self.wait_ticks = 4
        self.ticks = self.wait_ticks


    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            print(kwargs)
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.image_sub = self.node.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.cv_bridge = CvBridge()


    def update(self):
            return common.Status.RUNNING


    def image_callback(self, msg):
        if self.blackboard.get("disable_sign_detection"):
            return

        #wait specified ticks
        if self.ticks > 0:
            self.ticks -= 1
            return
        else:
            self.ticks = self.wait_ticks

        try:
            # convert "/image_raw" stream to image with is compatible to open-cv
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # start of sign detection
            detected_sign = self.detect_sign(cv_image)

            # publish detected sign
            if detected_sign is not None:
                if detected_sign.startswith(SignType.BARRIER.value) and self.count < 30:
                    self.count += 5
                if detected_sign.startswith(SignType.TRAIN.value):
                    self.blackboard.set("train_sign_detected", True)
                if detected_sign.startswith(SignType.PARK.value):
                    self.blackboard.set("park_sign_detected", True)
                    self.blackboard.set("disable_sign_detection", True)
                    self.blackboard.set("follow_line_white", False)
                    self.blackboard.set("h_line_detection_activated", True)
                    self.blackboard.set("h_line_detection_counter", 250)
                    self.blackboard.set("is_parking", True)

            if detected_sign is None or not detected_sign.startswith(SignType.BARRIER.value):
                if self.count > 0:
                    self.count -= 1

            if self.blackboard.get("train_sign_detected") and self.count >= 7 and not self.barrier_closed:
                self.barrier_closed = True
                self.blackboard.set("follow_line", False)
                self.blackboard.set("barrier_closed_detected", True)

            if self.blackboard.get("train_sign_detected") and self.barrier_closed and self.count < 4:
                self.barrier_closed = False
                self.count = 0
                self.blackboard.set("follow_line", True)
                self.blackboard.set("barrier_open_detected", True)
                self.blackboard.set("yellow_counter", 300)

            self.feedback_message = "barrier count: {}".format(self.count)


            # show video stream with found point
            cv2.imshow("Sign Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            print(str(e))

    def detect_sign(self, image):
        # Initialize SIFT
        sift = cv2.SIFT_create()

        # Define ROI
        h, w = image.shape[:2]
        #roi = image[h // 4:3 * h // 4, w // 4:3 * w // 4] #ROI in the center of the image
        roi = image[h // 4:3 * h // 4, w // 4:w] #ROI in center right of the image

        # For loop for every reference image
        for sign_name, reference_image in self.reference_images.items():
            # Detection with SIFT
            kp1, des1 = sift.detectAndCompute(reference_image, None)
            kp2, des2 = sift.detectAndCompute(roi, None)

            # Matching with FLANN
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks=50)
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des1, des2, k=2)

            # Filter matches using the Lowe's ratio test
            good_matches = [m for m, n in matches if m.distance < 0.7 * n.distance]

            # If enough good matches, estimate homography
            #print(len(good_matches))
            if len(good_matches) >= self.min_match_count:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                matchesMask = mask.ravel().tolist()
                #print(len(M))

                if M is not None:
                    # Paint matches into the image
                    for i, match in enumerate(good_matches):
                        if matchesMask[i]:
                            # Get coordinates
                            (x1, y1) = kp1[match.queryIdx].pt
                            (x2, y2) = kp2[match.trainIdx].pt

                            # Paint matches
                            cv2.circle(image, (int(x2 + w // 4), int(y2 + h // 4)), 5, (0, 255, 0), 2)
                    return sign_name

        return None

def main(args=None):
    rclpy.init(args=args)

    # Create a node
    node = SignDetection()
    node.get_logger().info("Node has been created")
    node.get_logger().info("Node is running")


    # Spin the node
    rclpy.spin(node)

    # Destroy the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()