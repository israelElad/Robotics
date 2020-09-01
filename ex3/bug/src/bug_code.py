#!/usr/bin/python

import rospy
import tf
import math
import sys
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Bug(object):

    # ctor
    def __init__(self, forward_speed, rotation_speed, min_scan_angle, max_scan_angle, min_dist_from_obstacle, goal_x,
                 goal_y):
        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed
        self.min_scan_angle = min_scan_angle / 180 * math.pi
        self.max_scan_angle = max_scan_angle / 180 * math.pi
        self.min_dist_from_obstacle = min_dist_from_obstacle
        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("/odom", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.mline_x_start = 0.0
        self.mline_y_start = 0.0
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.state = self.state_dict.ROTATE
        self.angle_precision = 0.2
        self.distance_to_goal_precision = 0.6
        self.distance_to_line_precision = 0.25
        self.m = 0
        self.b = 0
        self.moveFromLine = 0

        self.follow_state = self.follow_state_dict.FIND_OBSTACLE
        self.current_angle = 0

        # wait for Gazebo to load
        time.sleep(15)

    class follow_state_dict:
        FIND_OBSTACLE = 0
        TURN_LEFT = 1
        FOLLOW_OBSTACLE = 2

    class state_dict:
        ROTATE = 0
        OBSTACLE = 1
        STRAIGHT = 2
        FINISH = 3

    # start moving the robot
    def start_moving(self):
        rate = rospy.Rate(2.0)
        isFirstTime = True
        self.calc_line_to_goal()

        while not rospy.is_shutdown():

            try:
                (trans, rot) = self.listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
                self.robot_x = trans[0]
                self.robot_y = trans[1]
                if isFirstTime:
                    self.mline_x_start = trans[0]
                    self.mline_y_start = trans[1]
                (roll, pitch, theta) = tf.transformations.euler_from_quaternion(rot)
                self.current_angle = theta
                self.bug2_alg()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                rospy.logerr("Service call failed: %s" % e)

            rate.sleep()

    def bug2_alg(self):
        if self.state == self.state_dict.ROTATE:
            rospy.loginfo("bug2 ROTATE")
            self.rotate_to_target()
        elif self.state == self.state_dict.OBSTACLE:
            rospy.loginfo("bug2 OBSTACLE")
            self.follow_obstacle()
        # straight on the m line until finish or obstacle
        elif self.state == self.state_dict.STRAIGHT:
            rospy.loginfo("bug2 STRAIGHT")
            self.go_straight()
        elif self.state == self.state_dict.FINISH:
            rospy.loginfo("bug2 FINISH")
            self.finish()

    def rotate_to_target(self):
        wanted_angle = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
        error_angle = wanted_angle - self.current_angle
        if (math.fabs(error_angle) > math.pi):
            error_angle = error_angle - (error_angle * 2 * math.pi) / (math.fabs(error_angle))

        if math.fabs(error_angle) > self.angle_precision:
            # rospy.loginfo("rotating to target. current angle:" + str(self.current_angle) + "   wanted angle=" + str(wanted_angle))
            self.move_msg = Twist()
            self.move_msg.angular.z = self.rotation_speed if error_angle > 0 else -self.rotation_speed
            self.command_pub.publish(self.move_msg)
        else:
            self.change_state(self.state_dict.STRAIGHT)

    def follow_obstacle(self):
        msg = Twist()
        # find the obstacle
        if (self.follow_state == self.follow_state_dict.FIND_OBSTACLE):
            rospy.loginfo("searching: follow state is FIND_OBSTACLE")
            msg.linear.x = 0.4
            msg.angular.z = -0.3
            self.moveFromLine += 1
        # turn left
        elif (self.follow_state == self.follow_state_dict.TURN_LEFT):
            rospy.loginfo("rotating: follow state is TURN_LEFT")
            msg.angular.z = 0.3
        # follow the obstacle
        elif (self.follow_state == self.follow_state_dict.FOLLOW_OBSTACLE):
            rospy.loginfo("following: follow state is FOLLOW_OBSTACLE")
            msg.linear.x = 0.5
            self.moveFromLine += 1
            # rospy.loginfo("moveFromLine: %s" % self.moveFromLine)
        self.command_pub.publish(msg)

    def go_straight(self):
        # rospy.loginfo("going straight")

        dist_to_goal = math.sqrt(pow(self.goal_y - self.robot_y, 2) + pow(self.goal_x - self.robot_x, 2))
        rospy.loginfo("dist to goal: %s" % dist_to_goal)
        if dist_to_goal > self.distance_to_goal_precision:
            msg = Twist()
            msg.linear.x = self.forward_speed
            msg.angular.z = 0
            self.command_pub.publish(msg)
        else:
            self.change_state(self.state_dict.FINISH)

    def finish(self):
        # rospy.loginfo("finish")
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.command_pub.publish(msg)

    def change_state(self, new_state):
        if self.state is not new_state:
            rospy.loginfo("old state: %s. new state: %s" % (self.state, new_state))
            self.state = new_state
            if new_state == self.state_dict.ROTATE:
                # immediate stop!
                msg = Twist()
                msg.linear.x = 0
                self.command_pub.publish(msg)

    def change_follow_obstacle_state(self, new_follow_state):
        if self.follow_state is not new_follow_state:
            rospy.loginfo("old follow state: %s. new follow state: %s" % (self.follow_state, new_follow_state))
            self.follow_state = new_follow_state

    def calc_line_to_goal(self):
        self.m = (self.robot_y - self.goal_y) / (self.robot_x - self.goal_x)
        self.b = self.goal_y - (self.goal_x * self.m)

    def is_on_line_to_target(self):
        return self.robot_y <= (self.m * self.robot_x + self.b) + self.distance_to_line_precision and self.robot_y >= (
                self.m * self.robot_x + self.b) - self.distance_to_line_precision

    # this function is called each time a laser reading occurs
    def scan_callback(self, scan_msg):
        min_in_regions = {
            'fright': min(scan_msg.ranges[0:240]),
            'front': min(scan_msg.ranges[241:399]),
            'fleft': min(scan_msg.ranges[400:640]),
        }
        m_front = min_in_regions['front']
        m_fleft = min_in_regions['fleft']
        m_fright = min_in_regions['fright']

        if math.isnan(m_fleft):
            m_fleft = 10
        if math.isnan(m_front):
            m_front = 10
        if math.isnan(m_fright):
            m_fright = 10

        # print min_in_regions
        min_dist = self.min_dist_from_obstacle
        if self.state == self.state_dict.STRAIGHT:
            if m_front < min_dist:
                # obstacle ahead!
                # rospy.loginfo("obstacle ahead! state=OBSTACLE")
                self.change_state(self.state_dict.OBSTACLE)
        if self.state == self.state_dict.OBSTACLE:
            # rospy.loginfo(self.is_on_line_to_target())

            if self.is_on_line_to_target() and self.moveFromLine > 5:
                self.change_state(self.state_dict.ROTATE)
                self.moveFromLine = 0


            elif m_front > min_dist:
                # no obstacle detected yet
                if m_fleft > min_dist and m_fright > min_dist:
                    self.change_follow_obstacle_state(self.follow_state_dict.FIND_OBSTACLE)
                # fright
                elif m_fleft > min_dist and m_fright < min_dist:
                    self.change_follow_obstacle_state(self.follow_state_dict.FOLLOW_OBSTACLE)
                # fleft
                elif m_fleft < min_dist and m_fright > min_dist:
                    self.change_follow_obstacle_state(self.follow_state_dict.FIND_OBSTACLE)
                # fleft, fright
                elif m_fleft < min_dist and m_fright < min_dist:
                    self.change_follow_obstacle_state(self.follow_state_dict.FIND_OBSTACLE)

            elif m_front < min_dist:
                # front
                if m_fleft > min_dist and m_fright > min_dist:
                    self.change_follow_obstacle_state(self.follow_state_dict.TURN_LEFT)
                # front, fright
                elif m_fleft > min_dist and m_fright < min_dist:
                    self.change_follow_obstacle_state(self.follow_state_dict.TURN_LEFT)
                # front, fleft
                elif m_fleft < min_dist and m_fright > min_dist:
                    self.change_follow_obstacle_state(self.follow_state_dict.TURN_LEFT)
                # front, fleft, fright
                elif m_fleft < min_dist and m_fright < min_dist:
                    self.change_follow_obstacle_state(self.follow_state_dict.TURN_LEFT)
