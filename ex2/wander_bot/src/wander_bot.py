#!/usr/bin/python3

import rospy
import math
import sys
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Wander_bot(object):
    #ctor
    def __init__(self, forward_speed, rotation_speed, min_scan_angle, max_scan_angle, min_dist_from_obstacle):
        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed
        self.min_scan_angle = min_scan_angle / 180 * math.pi
        self.max_scan_angle = max_scan_angle / 180 * math.pi
        self.min_dist_from_obstacle = min_dist_from_obstacle
        self.shouldStop=False
        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan",LaserScan, self.scan_callback, queue_size=1)
        self.beginning=True
        #wait for Gazebo to load
        time.sleep(25)

    #start moving the robot
    def start_moving(self):
        rate = rospy.Rate(10)
        #rotate in place for 20 seconds
        t_end = time.time() + 20
        while time.time() < t_end:
            #rotate
            self.move_msg = Twist()
            self.move_msg.angular.z=self.rotation_speed
            self.command_pub.publish(self.move_msg)
        
        #stop rotating and advance
        self.move_msg.angular.z=0
        while not rospy.is_shutdown():
            self.move_forward()
            rate.sleep()

    #moves the robot forward
    def move_forward(self):
        if(not self.shouldStop):       
            self.move_msg = Twist()
            self.move_msg.linear.x = self.forward_speed
            self.command_pub.publish(self.move_msg)

    #stop moving the robot forward
    def stop_moving_forward(self):
        self.move_msg = Twist()
        self.move_msg.linear.x = 0
        self.command_pub.publish(self.move_msg)
        self.shouldStop=True

    #this function is called each time a laser reading occurs
    def scan_callback(self, scan_msg):
        #iterate over the distances matrix received from the laser sensors
        for dist in scan_msg.ranges:
            #obstacle detected!
            if dist < self.min_dist_from_obstacle:
                #rotate
                self.stop_moving_forward()
                self.move_msg = Twist()
                self.move_msg.angular.z=self.rotation_speed
                self.command_pub.publish(self.move_msg)
                return
        
        #the way ahead is clear
        self.move_msg = Twist()
        self.move_msg.angular.z=0
        self.command_pub.publish(self.move_msg)
        self.shouldStop=False

