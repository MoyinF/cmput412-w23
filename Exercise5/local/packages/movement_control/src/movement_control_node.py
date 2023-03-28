#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32
from turbojpeg import TurboJPEG
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped, VehicleCorners, LEDPattern
from duckietown_msgs.srv import ChangePattern, SetCustomLEDPattern
from movement_control.srv import DetectionService
from dt_apriltags import Detector, Detection
import yaml
import random


ROAD_MASK = [(20, 60, 0), (50, 255, 255)]  # for yellow mask
DEBUG = False
ENGLISH = False


class MovementControlNode(DTROS):

    def __init__(self, node_name):
        super(MovementControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = rospy.get_param("~veh")

        # Publishers
        self.pub_mask = rospy.Publisher(
            "/" + self.veh + "/output/image/mask/compressed", CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher(
            "/" + self.veh + "/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.image_pub = rospy.Publisher(f'/{self.veh}/{node_name}/augmented_image/image/compressed', CompressedImage, queue_size=16)
        self.debug_pub = rospy.Publisher(f'/{self.veh}/{node_name}/debug_image/image/compressed', CompressedImage, queue_size=16)
        self.pub_debug_img_bool = True

        # Subscribers
        self.sub_camera = rospy.Subscriber(
            "/" + self.veh + "/camera_node/image/compressed",
            CompressedImage,
            self.img_callback,
            queue_size=1,
            buff_size="20MB")
        self.image_msg = None

        # Services proxies
        rospy.wait_for_service(f'/{self.veh}/digit_detection_node/digit_detection_service')
        self.digit_detection_service = rospy.ServiceProxy(f'/{self.veh}/digit_detection_node/digit_detection_service', DetectionService)

        # image processing tools
        self.bridge = CvBridge()
        self.jpeg = TurboJPEG()

        # info from subscribers
        self.intersection_detected = None

        # find calibration parameters for detecting apriltags
        camera_intrinsic_dict = self.readYamlFile(
            f'/data/config/calibrations/camera_intrinsic/{self.veh}.yaml')

        self.K = np.array(
            camera_intrinsic_dict["camera_matrix"]["data"]).reshape((3, 3))
        self.R = np.array(
            camera_intrinsic_dict["rectification_matrix"]["data"]).reshape((3, 3))
        self.DC = np.array(
            camera_intrinsic_dict["distortion_coefficients"]["data"])
        self.P = np.array(
            camera_intrinsic_dict["projection_matrix"]["data"]).reshape((3, 4))
        self.h = camera_intrinsic_dict["image_height"]
        self.w = camera_intrinsic_dict["image_width"]

        f_x = camera_intrinsic_dict['camera_matrix']['data'][0]
        f_y = camera_intrinsic_dict['camera_matrix']['data'][4]
        c_x = camera_intrinsic_dict['camera_matrix']['data'][2]
        c_y = camera_intrinsic_dict['camera_matrix']['data'][5]
        self.camera_params = [f_x, f_y, c_x, c_y]

        # initialize apriltag detector
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=1,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.at_distance = 0
        self.at_locations = {
            200: (0.17, 0.17),  # (x, y) position in world frame
            201: (1.65, 0.17),
            94: (1.65, 2.84),
            93: (0.17, 2.84),
            153: (1.75, 1.252),
            133: (1.253, 1.755),
            58: (0.574, 1.259),
            62: (0.075, 1.755),
            169: (0.574, 1.755),
            162: (1.253, 1.253)
        }
        self.intersections = {
            133: 'INTER_R',  # T intersection with right-turn option
            153: 'INTER_L',  # T intersection with left-turn option
            62: 'INTER_L',  # T intersection with left-turn option
            58: 'INTER_R',  # T intersection with right-turn option
            162: 'STOP',  # Stop sign
            169: 'STOP'   # Stop sign
        }

        # set of found digits and apriltags
        self.ats_found = {}
        # tag id for debugging
        self.at_tag_id = ""
        # list of digits left
        self.digits_list = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]

        # apriltag detection filters
        self.decision_threshold = 7
        self.z_threshold = 0.45

        # PID Variables for driving
        self.proportional = None
        self.offset = 170
        self.velocity = 0.25
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        self.P = 0.025
        self.D = -0.0035
        self.last_error = 0
        self.last_time = rospy.get_time()
        self.calibration = 0.5

        self.loginfo("Initialized")

        # Shutdown hook
        rospy.on_shutdown(self.hook)

    def img_callback(self, msg):
        self.image_msg = msg

    def run(self):
        rate = rospy.Rate(8)  # 8hz
        i = 0
        while not rospy.is_shutdown():
            i += 1

            if i % 2 == 0:
                self.image_pub.publish(self.image_msg)
            if i % 4 == 0 or self.intersection_detected:
                if self.image_msg is not None:
                    self.detect_digits(self.image_msg)
                    self.detect_lane(self.image_msg)

            if self.intersection_detected:
                self.intersection_sequence()
            self.drive()

            rate.sleep()

    def drive(self):
        if self.proportional is None:
            self.twist.omega = 0
        else:
            # P Term
            P = -self.proportional * self.P

            # D Term
            d_error = (self.proportional - self.last_error) / \
                (rospy.get_time() - self.last_time)
            self.last_error = self.proportional
            self.last_time = rospy.get_time()
            D = d_error * self.D

            self.twist.v = self.velocity
            self.twist.omega = P + D

        self.vel_pub.publish(self.twist)

    def intersection_sequence(self):

        rospy.loginfo("INTERSECTION DETECTED. at {}".format(str(self.at_distance)))

        # advance to intersection
        self.pub_straight(linear=0)
        wait_time = self.at_distance * 1.5 # seconds
        self.pass_time(wait_time)

        # stop at the intersection
        self.stop()
        self.pass_time(3)

        # advance into intersection
        self.pub_straight(linear=0)
        self.pass_time(2)

        # turn at random
        if self.intersection_detected == 'INTER_R' and random.random() < 0.5:
            self.right_turn()
        elif self.intersection_detected == 'INTER_L' and random.random() < 0.5:
            self.left_turn()
        elif self.intersection_detected == 'STOP':
            if random.random() < 0.5:
                self.left_turn()
            else:
                self.right_turn()

        # exit intersection
        self.pub_straight(linear=0)
        self.pass_time(1.0)

    def right_turn(self):
        rospy.loginfo("TURNING RIGHT")
        self.twist.v = 0
        self.twist.omega = -12
        self.vel_pub.publish(self.twist)
        start_time = rospy.get_time()
        while rospy.get_time() < start_time + 1.5:
            continue
        self.stop()

    def left_turn(self):
        rospy.loginfo("TURNING LEFT")
        self.twist.v = 0
        self.twist.omega = 12
        self.vel_pub.publish(self.twist)
        start_time = rospy.get_time()
        while rospy.get_time() < start_time + 1.5:
            continue
        self.stop()

    def pub_straight(self, linear=None):
        self.twist.v = self.velocity
        if linear is not None:
            self.twist.omega = linear
        else:
            self.twist.omega = self.calibration
        self.vel_pub.publish(self.twist)

    def stop(self):
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)

    def pass_time(self, t):
        start_time = rospy.get_time()
        while rospy.get_time() < start_time + t:
            continue

    def detect_lane(self, msg):
        img = self.jpeg.decode(msg.data)

        crop = img[:, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        crop = cv2.bitwise_and(crop, crop, mask=mask)
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Search for lane in front
        max_area = 20
        max_idx = -1
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > max_area:
                max_idx = i
                max_area = area

        if max_idx != -1:
            M = cv2.moments(contours[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.proportional = cx - int(crop_width / 2) + self.offset
                if DEBUG:
                    cv2.drawContours(crop, contours, max_idx, (0, 255, 0), 3)
                    cv2.circle(crop, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass
        else:
            self.proportional = None

        # debugging
        if self.pub_debug_img_bool:
            rect_img_msg = CompressedImage(
                format="jpeg", data=self.jpeg.encode(crop))
            self.pub_mask.publish(rect_img_msg)

    def detect_digits(self, img_msg):

        # convert image into cv2 type
        cv_image = None
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
            self.log(e)
            return []

        # undistort the image
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            self.K, self.DC, (self.w, self.h), 0, (self.w, self.h))
        image_np = cv2.undistort(cv_image, self.K, self.DC, None, newcameramtx)

        # convert the image to black and white
        image_gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

        # detect tags present in image
        tags = self.at_detector.detect(
            image_gray, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=0.065)

        # get the closest-detected tag
        closest_tag_z = 1000
        closest = None

        for tag in tags:
            # ignore distant tags and tags with bad decoding
            z = tag.pose_t[2][0]
            if tag.decision_margin < self.decision_threshold or z > self.z_threshold:
                continue

            # update the closest-detected tag if needed
            if z < closest_tag_z:
                closest_tag_z = z
                closest = tag

        if closest:
            # create a square above the apriltag
            x_coords = [corner[0].astype(int) for corner in closest.corners]
            y_coords = [corner[1].astype(int) for corner in closest.corners]
            y_coords = y_coords - (y_coords[1] - y_coords[2] + 10) # Shift bounding box upward

            x_min = min(x_coords) - 10
            x_max = max(x_coords) + 10
            y_min = min(y_coords) - 20
            y_max = max(y_coords) + 10

            digit = "None"

            # if closest.tag_id in self.ats_found:
                # digit = self.ats_found[closest.tag_id]

            # else:
            # stop the robot
            self.stop()
            # grab a new camera image
            self.pass_time(1.5)
            new_image = self.bridge.compressed_imgmsg_to_cv2(self.image_msg)
            new_image = cv2.undistort(new_image, self.K, self.DC, None, newcameramtx)

            # run digit detection on the cropped image
            cropped_image = new_image[y_min:y_max, x_min:x_max]

            if len(cropped_image) == 0:
                rospy.loginfo("Empty crop.")
                return

            digit = self.get_digit(cropped_image)

            # update tag id
            self.at_tag_id = str(closest.tag_id)

            # print the digit and coresponding apriltag location to the console
            tag_x, tag_y = self.at_locations[closest.tag_id]
            rospy.loginfo(f"DIGIT DETECTED\n\tdigit: {str(digit)}\n\tapriltag: {str(closest.tag_id)}\n\tposition (x,y): ({str(tag_x)}, {str(tag_y)})\n")


            # terminate if all digits have been found
            self.ats_found[closest.tag_id] = digit
            if digit in self.digits_list:
                rospy.loginfo("{} apriltags found".format(str(len(self.ats_found))))
                self.digits_list.remove(digit)
                rospy.loginfo("Remaining digits: {}".format(str(self.digits_list)))

            # label the apriltag and digit
            self.labelDigit(image_np, digit, (x_min, y_min), (x_max, y_max))
            self.labelTag(image_np, closest)

            # publish new compressed image
            augmented_image_msg = CompressedImage()
            augmented_image_msg.header.stamp = rospy.Time.now()
            augmented_image_msg.format = "jpeg"
            augmented_image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()

            self.image_pub.publish(augmented_image_msg)
            self.pass_time(1.5)


            if len(self.ats_found) == 10 or len(self.digits_list) == 0:
                rospy.loginfo("{} apriltags found. Shutting down.".format(str(len(self.ats_found))))
                rospy.loginfo("Remaining digits: {}".format(str(self.digits_list)))
                rospy.signal_shutdown("Program terminating.")

            # continue driving straight # derivative kick avoidance
            # self.pub_straight()

            # check if the apriltag is an intersection
            if closest.tag_id in self.intersections:
                self.at_distance = closest.pose_t[2][0]
                self.intersection_detected = self.intersections[closest.tag_id]
            else:
                self.intersection_detected = None
        else:
            self.intersection_detected = None



    def get_digit(self, image):

        image_msg = CompressedImage(format="jpeg", data=cv2.imencode('.jpg', image)[1].tobytes())

        if self.pub_debug_img_bool:
            self.debug_pub.publish(image_msg)

        response = "None"
        try:
            response = self.digit_detection_service(image_msg)
            response = str(response.digit.data)
        except Exception as e:
            rospy.loginfo(f"Encountered exception while calling digit detection service: {str(e)}")

        return response

    def labelTag(self, image, tag):

        # label the apriltag
        text = str(tag.tag_id)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = 0.8
        thickness = 2

        textsize = cv2.getTextSize(text, font, font_size, thickness)[0]
        text_x = tag.center[0].astype(int) - (textsize[0] // 2)
        text_y = tag.center[1].astype(int) + (textsize[1] // 2)

        cv2.putText(image, text, (text_x, text_y), font, font_size, (0, 255, 0), thickness)

        # draw a border box around the apriltag
        for i in range(len(tag.corners)):
            point_x = tuple(tag.corners[i-1, :].astype(int))
            point_y = tuple(tag.corners[i, :].astype(int))

            cv2.line(image, point_x, point_y, (0, 255, 0), 5)

        return image

    def labelDigit(self, image, digit, pt1, pt2):

        # label the digit
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = 1.5
        thickness = 3

        cv2.putText(image, str(digit), pt2, font, font_size, (255, 0, 0), thickness)

        # draw a border box around the digit
        cv2.rectangle(image, pt1, pt2, (255, 0, 0), thickness)

        return image

    def hook(self):
        print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)
        for i in range(8):
            self.vel_pub.publish(self.twist)

    def readYamlFile(self, fname):
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         % (fname, exc), type='fatal')
                rospy.signal_shutdown()
                return


if __name__ == "__main__":
    node = MovementControlNode("movement_control_node")
    node.run()
