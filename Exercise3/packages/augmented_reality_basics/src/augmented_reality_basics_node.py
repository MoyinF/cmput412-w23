#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from augmenter import Augmenter
from cv_bridge import CvBridge


class AugmentedRealityBasicsNode(DTROS):
    def __init__(self, node_name):
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.veh = rospy.get_param("~veh")
        self.map_file = rospy.get_param("~map_file")
        
        # Subscribers
        self.sub = rospy.Subscriber(f'{self.veh}/camera_node/image/compressed', CompressedImage, self.project_cb)
        
        # Publishers
        self.pub = rospy.Publisher(f'{self.veh}/augmented_reality_basics_node/{self.map_file}/image/compressed', CompressedImage, queue_size=10)
        
        # Properties
        self.augmenter = Augmenter()
    
    def project_cb(self,msg):
        br = CvBridge()
        self.raw_image = br.compressed_imgmsg_to_cv2(msg)
        dis = self.augmenter.process_image(self.raw_image)
        render = self.augmenter.render_segments(dis)
        result = br.cv2_to_compressed_imgmsg(render,dst_format='jpg')
        self.pub.publish(result)


if __name__ == '__main__':
    node = AugmentedRealityBasicsNode('augmented_reality_basics_node')
    rospy.spin()
