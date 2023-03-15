#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2
from renderClass import Renderer

import rospy
import yaml
import sys
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import rospkg 

from dt_apriltags import Detector, Detection


"""

This is a template that can be used as a starting point for the CRA1 exercise.
You need to project the model file in the 'models' directory on an AprilTag.
To help you with that, we have provided you with the Renderer class that render the obj file.

"""

class ARNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ARNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")

        rospack = rospkg.RosPack()
        # Initialize an instance of Renderer giving the model in input.
        self.renderer = Renderer(rospack.get_path('augmented_reality_apriltag') + '/src/models/duckie.obj')
        
        # Publishers
        self.image_pub = rospy.Publisher(f'/{self.veh}/{node_name}/augmented_image/image/compressed', CompressedImage, queue_size=16)

        # Subscribers
        self.image_sub = rospy.Subscriber(f'/{self.veh}/camera_node/image/compressed', CompressedImage, self.callback)

        # initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # find the calibration parameters
        camera_extrinsic_dict =  self.readYamlFile(f'/data/config/calibrations/camera_extrinsic/{self.veh}.yaml')
        homography = camera_extrinsic_dict['homography']
        
        self.H = [homography[0:3], homography[3:6], homography[6:9]]
        self.Hinv = np.linalg.inv(self.H)
        
        camera_intrinsic_dict =  self.readYamlFile(f'/data/config/calibrations/camera_intrinsic/{self.veh}.yaml')
        
        self.K = np.array(camera_intrinsic_dict["camera_matrix"]["data"]).reshape((3, 3))
        
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
                           
        # for storing image
        self.image = None


    def callback(self, image_msg):
        
        self.image = image_msg

    
    def projection_matrix(self, K, H):
        """
            Write here the compuatation for the projection matrix, namely the matrix
            that maps the camera reference frame to the AprilTag reference frame.
            
            DO NOT SUBMIT THIS CODE
            
            This code comes from https://github.com/liqianxi/503_ex3/blob/exercise3/packages/augmented_reality_apriltag/src/augmented_reality_apriltag.py
        """

        # find R_1, R_2 and t
        Kinv = np.linalg.inv(K)
        r1r2t = np.matmul(Kinv, H) # r1r2t = [r_1 r_2 t]
        r1r2t = r1r2t / np.linalg.norm(r1r2t[:, 0])
        r_1 = r1r2t[:, 0]
        r_2 = r1r2t[:, 1]
        t = r1r2t[:, 2]

        # Find v_3 vector othogonal to v_1 and v_2
        r_3 = np.cross(r_1, r_2)

        # Reconstruct R vector
        R = np.column_stack((r_1, r_2, r_3))

        # Use SVD to make R into an orthogonal matrix
        _, U, Vt = cv2.SVDecomp(R)
        R = U @ Vt

        # Combine R, t and K to find P
        buff = np.column_stack((R, t)) # buff = [r_1 r_2 r_3 t]
        P = K @ buff

        return P

    def readImage(self, msg_image):
        """
            Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            self.log(e)
            return []

    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def run(self):
        """
        DO NOT SUBMIT THIS CODE
        
        Much of the code is adapted from https://github.com/liqianxi/503_ex3/blob/exercise3/packages/augmented_reality_apriltag/src/augmented_reality_apriltag.py
        """
        
        # publish image 5 times per second
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown():

            if self.image:
                # extract the image message to a cv image
                image_np = self.bridge.compressed_imgmsg_to_cv2(self.image)

                # detect apriltag and extract its reference frame
                image_gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
                tags = self.at_detector.detect(image_gray, estimate_tag_pose=False, camera_params=self.camera_params, tag_size=0.065)

                for tag in tags:
                    H = tag.homography # assume only 1 tag in image

                    # Find transformation from april tag to target image frame
                    P = self.projection_matrix(self.K, H)

                    # Project model into image frame
                    image_np = self.renderer.render(image_np, P)

                # make new CompressedImage to publish
                augmented_image_msg = CompressedImage()
                augmented_image_msg.header.stamp = rospy.Time.now()
                augmented_image_msg.format = "jpeg"
                augmented_image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()

                # Publish new image
                self.image_pub.publish(augmented_image_msg)
                
            rate.sleep()

    def onShutdown(self):
        super(ARNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = ARNode(node_name='augmented_reality_apriltag_node')
    # run the node
    camera_node.run()
