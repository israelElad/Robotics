#!/usr/bin/python

import math
import rospy
import globals
from geometry_msgs.msg import Twist

pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

rotate_nothing = 0
rotate_quarter = 90
rotate_2_quarters = 180
rotate_3_quarters = 270


def move_along_path(path, robot_diameter):
    move(robot_diameter * 2)
    globals.direction = globals.direction_up

    for idx_in_path in range(0, len(path) - 1):
        this_direction = calc_direction(path[idx_in_path], path[idx_in_path + 1])
        rotation_angle = calc_rotation_angle(this_direction)
        globals.direction = this_direction

        rotate_in_place(rotation_angle)
        move(robot_diameter)


def calc_direction(this_p, next_p):
    if next_p[0] != this_p[0]:
        if next_p[0] <= this_p[0]:
            return globals.direction_down
        else:
            return globals.direction_up
    else:
        if next_p[1] <= this_p[1]:
            return globals.direction_left
        else:
            return globals.direction_right


def calc_rotation_angle(current_direction):
    # if the current direction is right
    if globals.direction == globals.direction_right:
        # and we want to go up
        if current_direction == globals.direction_up:
            return rotate_3_quarters
        elif current_direction == globals.direction_right:
            return rotate_nothing
        elif current_direction == globals.direction_down:
            return rotate_quarter
        else:  # globals.direction_left
            return rotate_2_quarters

    elif globals.direction == globals.direction_up:
        if current_direction == globals.direction_up:
            return rotate_nothing
        elif current_direction == globals.direction_right:
            return rotate_quarter
        elif current_direction == globals.direction_down:
            return rotate_2_quarters
        else:  # globals.direction_left
            return rotate_3_quarters

    elif globals.direction == globals.direction_down:
        if current_direction == globals.direction_up:
            return rotate_2_quarters
        elif current_direction == globals.direction_right:
            return rotate_3_quarters
        elif current_direction == globals.direction_down:
            return rotate_nothing
        else:  # globals.direction_left
            return rotate_quarter

    else:  # globals.direction == globals.direction_left:
        if current_direction == globals.direction_up:
            return rotate_quarter
        elif current_direction == globals.direction_right:
            return rotate_2_quarters
        elif current_direction == globals.direction_down:
            return rotate_3_quarters
        else:  # globals.direction_left
            return rotate_nothing


def rotate_in_place(angle):
    speed = 6
    angular_speed = speed * 2 * math.pi / 360
    relative_angle = angle * 2 * math.pi / 360

    vel_msg = Twist()
    vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
    vel_msg.angular.z = -abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    curr_angel = 0

    while curr_angel < relative_angle:
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        curr_angel = angular_speed * (t1 - t0)

    vel_msg.angular.z = 0
    pub.publish(vel_msg)


def move(distance):
    speed = 0.15

    vel_msg = Twist()
    vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0
    vel_msg.linear.x = abs(speed)
    t0 = rospy.Time.now().to_sec()
    curr_distance = 0

    while curr_distance < distance:
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        curr_distance = speed * (t1 - t0)

    vel_msg.linear.x = 0
    pub.publish(vel_msg)
