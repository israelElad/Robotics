#!/usr/bin/python

import rospy, sys
from bug_code import Bug
if __name__ == "__main__":
    rospy.init_node("bug_node", argv=sys.argv)
    forward_speed = 0.5
    if rospy.has_param('~forward_speed'):
        forward_speed = rospy.get_param('~forward_speed')
    rotation_speed = 0.5
    if rospy.has_param('~rotation_speed'):
        rotation_speed = rospy.get_param('~rotation_speed')
    min_scan_angle = -30
    if rospy.has_param('~min_scan_angle'):
        min_scan_angle = rospy.get_param('~min_scan_angle')
    max_scan_angle = 30
    if rospy.has_param('~max_scan_angle'):
        max_scan_angle = rospy.get_param('~max_scan_angle')
    min_dist_from_obstacle=0.5
    if rospy.has_param('~min_dist_from_obstacle'):
        min_dist_from_obstacle = rospy.get_param('~min_dist_from_obstacle')
    goal_x=-7.0
    if rospy.has_param('~goal_x'):
        goal_x = rospy.get_param('~goal_x')
    goal_y=-5.0
    if rospy.has_param('~goal_y'):
        goal_y = rospy.get_param('~goal_y')

    my_Bug = Bug(forward_speed,rotation_speed, min_scan_angle, max_scan_angle, min_dist_from_obstacle,goal_x,goal_y)
    my_Bug.start_moving()
