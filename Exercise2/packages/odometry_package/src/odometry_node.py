#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='left')
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='right')
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'/{self.veh_name}/wheel_pub_sub_node/integrated_distance_left', Float32, queue_size=10)
        self.pub_integrated_distance_right = rospy.Publisher(f'/{self.veh_name}/wheel_pub_sub_node/integrated_distance_right', Float32, queue_size=10)
        
        # vehicle state
        self.prev_left = 0
        self.prev_right = 0
        
        self.first_message_left = True
        self.first_message_right = True
        self.initial_left = 0
        self.initial_right = 0
        
        self.left_distance = 0
        self.right_distance = 0
        
        self.vel_left = 0
        self.vel_right = 0

        self.log("Initialized")

    def cb_encoder_data(self, msg, wheel):
        """ Update encoder distance information from ticks.
        """
        # retreive ticks and resolution from message
        ticks = msg.data
        resolution = msg.resolution
        
        # Check if it's the first received message and store initial encoder value
        if wheel == 'left' and self.first_message_left == True:
            self.first_message_left = False
            self.initial_left = ticks
        if wheel == 'right' and self.first_message_right ==True:
            self.first_message_right = False
            self.initial_right = ticks
        
        # Compute total distance traveled by the left wheel
        if wheel == 'left':
            rel_ticks = ticks - self.initial_left
            diff_ticks = np.abs(rel_ticks - self.prev_left)
            dist = (2 * np.pi * self._radius * diff_ticks / resolution)
            
            # Accumulate distance and publish it
            if self.vel_left >= 0:
            	self.left_distance += dist
            else:
                self.left_distance -= dist
                
            self.pub_integrated_distance_left.publish(self.left_distance)
            
            self.prev_left = rel_ticks

        # Compute total distance traveled by the right wheel
        elif wheel == 'right':
            rel_ticks = ticks - self.initial_right
            diff_ticks = np.abs(rel_ticks - self.prev_right)
            dist = (2 * np.pi * self._radius * diff_ticks / resolution)
            
            # Accumulate distance and publish it
            if self.vel_right >= 0:
            	self.right_distance += dist
            else:
                self.right_distance -= dist
                
            self.pub_integrated_distance_right.publish(self.right_distance)
            
            self.prev_right = rel_ticks

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        # retreive wheel velocities from message
        self.vel_left = msg.vel_left
        self.vel_right = msg.vel_right

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
