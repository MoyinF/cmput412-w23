#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class CameraPubSubNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraPubSubNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # construct publisher
        self.pub = rospy.Publisher('output_image', CompressedImage, queue_size=10)
        
        # construct subscriber
        self.sub = rospy.Subscriber('input_image', CompressedImage, self.callback)
        
        # define object attributs
        self.image_data = None
        
        

    def run(self):
        # publish image every 1 second
        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            rospy.loginfo("Publishing image")
            if self.image_data:
                self.pub.publish(self.image_data)
            rate.sleep()

    def callback(self, data):
        self.image_data = data
        
        np_arr = np.fromstring(data.data, np.uint8)
        img = cv2.imdecode(np_arr, 1)
        
        rospy.loginfo("Received image with shape {}".format(img.shape))

if __name__ == '__main__':
    # create the node
    node = CameraPubSubNode(node_name='camera_pub_sub_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()


