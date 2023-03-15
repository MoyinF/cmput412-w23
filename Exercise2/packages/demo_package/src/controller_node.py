#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Header, Float32, ColorRGBA
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
import time

class ControllerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ControllerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh_name = rospy.get_namespace().strip("/")

        # Subscribers
        self.sub_distance_left = rospy.Subscriber(f'/{self.veh_name}/odometry_node/integrated_distance_left', Float32, self.cb_distance_left)
        self.sub_distance_right = rospy.Subscriber(f'/{self.veh_name}/odometry_node/integrated_distance_right', Float32, self.cb_distance_right)

        # Publishers
        self.pub_wheel_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)

        # Services
        rospy.wait_for_service(f'/{self.veh_name}/led_service_node/led_service')
        self.led_service = rospy.ServiceProxy(f'/{self.veh_name}/led_service_node/led_service', ChangePattern)

        # vehicle speeds
        self.x_speed = 0.3
        self.turn_speed = 0.5
        
        # wheel calibration constants for csc22906
        self.slow_down = 0.13 # for slowing down turns during rotations
        self.speed_up = 0.0525 # for speeding up the lagging wheel

        # wheel distances
        self.distance_left = 0
        self.distance_right = 0

        self.l = 0.05 # meters between the center of the wheel and the robot rotation
        self.r = 0.033	# radius of the wheel in meteres

        # set shutdown behaviour
        rospy.on_shutdown(self.stop)


    def run(self):
        self.real_run()

    def real_run(self):
        self.state_1(5)
        self.state_2()
        self.state_1(5)
        self.state_3()
        self.state_1(5)
        self.state_4()
        self.complete()

    def state_1(self, seconds):
        # change LEDs and wait for 5 seconds
        self.publish_leds("RED")
        time.sleep(seconds)

    def state_2(self):
        # change LEDs, move 90 degrees and forward 3 times
        self.publish_leds("GREEN")
        self.first_right_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)

    def state_3(self):
        # change LEDs and return to initial position
        self.publish_leds("BLUE")
        self.right_turn(np.pi/2)
        self.backward(1.25)
        self.stop()

    def state_4(self):
        # change LEDs and move in circular motion
        self.publish_leds("WHITE")
        self.forward(0.625) # move forward to be tangential to the circle
        self.circular_motion(0.3, 0.4)
        self.circular_motion(0.3, 0.4)
        
    def complete(self):
        # go through a flashing sequence to indicate completion
        seconds = 0.4
        self.publish_leds("GREEN")
        time.sleep(seconds)
        self.publish_leds("BLUE")
        time.sleep(seconds)
        self.publish_leds("RED")
        time.sleep(seconds)
        self.publish_leds("WHITE")
        time.sleep(seconds)
        self.publish_leds("GREEN")
        time.sleep(seconds)
        self.publish_leds("BLUE")
        time.sleep(seconds)
        self.publish_leds("RED")
        time.sleep(seconds)
        self.publish_leds("WHITE")
        time.sleep(seconds)

    def forward(self, total_distance):
        starting_distance = (self.distance_left + self.distance_right) / 2
        current_distance = starting_distance

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = self.x_speed
        msg.vel_right = self.x_speed + self.speed_up # csc22906

        while current_distance - starting_distance < total_distance:
            current_distance = (self.distance_left + self.distance_right) / 2
            self.pub_wheel_commands.publish(msg)

        self.stop()

    def backward(self, total_distance):
        starting_distance = (self.distance_left + self.distance_right) / 2
        current_distance = starting_distance

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = -self.x_speed
        msg.vel_right = -self.x_speed - self.speed_up

        rospy.loginfo("left:" + str(msg.vel_left) + " right: " + str(msg.vel_right))

        while starting_distance - current_distance < total_distance:
            current_distance = (self.distance_left + self.distance_right) / 2
            self.pub_wheel_commands.publish(msg)

        self.stop()

    def rot_dist(self, angle):
        # angle is in radians
        # r * theta is formula for arc length
        return angle * self.l

    def left_turn(self, angle):
        # total_distance is how far each wheel has to travel
        total_distance = self.rot_dist(angle)

        starting_distance_l = self.distance_left
        starting_distance_r = self.distance_right
        current_distance = 0

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()

        msg.vel_left = -self.turn_speed
        msg.vel_right = self.turn_speed + self.slow_down # csc22906

        while current_distance < total_distance:
            rospy.loginfo("left_turn current_distance " + str(current_distance))
            current_distance = (abs(starting_distance_l - self.distance_left) + abs(starting_distance_r - self.distance_right))/2
            self.pub_wheel_commands.publish(msg)
        self.stop()

    def right_turn(self, angle):
        total_distance = self.rot_dist(angle)

        starting_distance_l = self.distance_left
        starting_distance_r = self.distance_right
        current_distance = 0

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()

        msg.vel_left = self.turn_speed
        msg.vel_right = -self.turn_speed

        while current_distance < total_distance:
            rospy.loginfo("right_turn current_distance " + str(current_distance))
            current_distance = (abs(starting_distance_l - self.distance_left) + abs(starting_distance_r - self.distance_right))/2
            self.pub_wheel_commands.publish(msg)
        self.stop()

    def first_right_turn(self, angle):
        total_distance = self.rot_dist(angle)

        starting_distance_l = 0
        starting_distance_r = 0
        current_distance = 0

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()

        msg.vel_left = self.turn_speed
        msg.vel_right = -self.turn_speed + self.slow_down # csc22906

        while current_distance < total_distance:
            rospy.loginfo("first_right_turn current_distance " + str(current_distance))
            current_distance = (abs(starting_distance_l - self.distance_left) + abs(starting_distance_r - self.distance_right))/2
            self.pub_wheel_commands.publish(msg)
        self.stop()

    def circular_motion(self, rad, speed):
        # for clockwise motion
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()

        # rad is center of rotation to center of duckiebot,
        # so need big & small radii for each wheel
        big_distance = 2 * np.pi * (rad + self.l)
        small_distance = 2 * np.pi * (rad - self.l)

        vel_ratio = (rad + self.l) / (rad - self.l)

        starting_distance_l = self.distance_left
        starting_distance_r = self.distance_right

        outer_vel = speed * vel_ratio
        inner_vel = speed

        # clockwise motion: left wheel is the outer wheel, so it needs to have the bigger distance
        msg.vel_left = outer_vel
        msg.vel_right = inner_vel

        while (self.distance_left - starting_distance_l < big_distance) or (self.distance_right - starting_distance_r < small_distance):
            self.pub_wheel_commands.publish(msg)
        self.stop()

    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.pub_wheel_commands.publish(msg)

    def publish_leds(self, color):
        try:
            response = self.led_service(String(color))
            rospy.loginfo(response)
        except Exception as e:
            rospy.loginfo("Failure to change LED pattern:" + str(e))

    def cb_distance_left(self, msg):
        self.distance_left = msg.data

    def cb_distance_right(self, msg):
        self.distance_right = msg.data
    
    def test_state_2(self):
        # old speeds
        # self.x_speed = 0.4
        # self.turn_speed = 0.15
        self.state_1(5)
        self.slow_down = 0.13 # for slowing down turns during rotations
        self.speed_up = 0.0525 # for speeding up the lagging wheel
        self.publish_leds("GREEN")
        self.first_right_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)

        self.state_1(5)
        self.slow_down = 0.13 # for slowing down turns during rotations
        self.speed_up = 0.055 # for speeding up the lagging wheel
        self.publish_leds("GREEN")
        self.first_right_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)
        self.left_turn(np.pi/2)
        self.forward(1.25)
        self.complete()
        
    def test_state_4(self):
        self.state_1(7)
        self.publish_leds("WHITE")
        self.circular_motion(0.3, 0.4) # small radius, current normal speed
        self.circular_motion(0.3, 0.4) # small radius, current normal speed
        self.stop()
        self.complete()
        

if __name__ == '__main__':
    # create the node
    node = ControllerNode(node_name='controller_node')
    # run node
    node.run()
    # keep spinning
    # rospy.spin()
    reason = "Program over. Exiting."
    rospy.signal_shutdown(reason)
