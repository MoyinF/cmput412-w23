#!/usr/bin/env python3

import rospy
import rospkg
import sys
#sys.path.append('./dt_msgs')

from duckietown.dtros import DTROS, NodeType

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Int32
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from eval import DigitPredictor

from digit_detection.srv import DetectionService, DetectionServiceResponse


class DigitDetectionNode(DTROS):

    def __init__(self, node_name):
        super(DigitDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = rospy.get_param("~veh")
        self.model_name = rospy.get_param("~model")

        # Services
        self.service = rospy.Service(f'/{self.veh}/digit_detection_node/digit_detection_service', DetectionService, self.detect_digit)

        # image processing tools
        self.bridge = CvBridge()

        # MLP model
        self.INPUT_H = 28
        self.INPUT_W = 28
        self.INPUT_DIM = self.INPUT_H * self.INPUT_W
        self.OUTPUT_DIM = 10

        self.rospack = rospkg.RosPack()
        model_path = self.rospack.get_path("digit_detection") + "/src/" + self.model_name

        self.predictor = DigitPredictor(model_path, self.INPUT_DIM, self.OUTPUT_DIM)

    def detect_digit(self, img_msg):
        # convert image into cv2 type
        cv_image = None
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg.img)
        except CvBridgeError as e:
            self.log(e)
            return []

        # reformat the image to the appropriate 28 * 28 size
        # cv_image = cv2.resize(cv_image, (self.INPUT_H, self.INPUT_W))
        # cv_image = self.mask_img(cv_image)

        # predict the digit
        digit = self.predictor.predict(cv_image)

        # return the result
        return DetectionServiceResponse(Int32(digit))

    def blue_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # blue colors
        lower_range = np.array([97, 66, 133])
        upper_range = np.array([108,255,255])
        mask = cv2.inRange(hsv, lower_range, upper_range)
        # With canny edge detection:
        edged = cv2.Canny(mask, 30, 200)

        return mask

    def mask_img(self, img):
        # take in cv_image # if img is filename: im = cv2.imread(f'{img}')
        im = self.blue_mask(img)

        # add line(s) around border to prevent floodfilling the digits
        cv2.line(im, (0,self.INPUT_W), (self.INPUT_H,self.INPUT_W), (255, 255, 255), 1)
        # cv2.line(im, (0,self.INPUT_W), (self.INPUT_H,self.INPUT_W), (255, 255, 255), 1)
        # cv2.line(im, (0,self.INPUT_W), (self.INPUT_H,self.INPUT_W), (255, 255, 255), 1)
        # cv2.line(im, (0,self.INPUT_W), (self.INPUT_H,self.INPUT_W), (255, 255, 255), 1)

        im = cv2.copyMakeBorder(im, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, value = 0)
        cv2.floodFill(im, None, (0, self.INPUT_W), 255)
        cv2.floodFill(im, None, (self.INPUT_H, 0), 255)
        cv2.floodFill(im, None, (0, 0), 255)
        cv2.floodFill(im, None, (self.INPUT_H, self.INPUT_W), 255)
        im = cv2.resize(im, (self.INPUT_H, self.INPUT_W))
        im = cv2.bitwise_not(im)

        return im

if __name__ == "__main__":
    node = DigitDetectionNode("digit_detection_node")
    rospy.spin()
