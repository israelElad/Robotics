#!/usr/bin/python3

import rospy, sys
from wander_bot import Wander_bot
if __name__ == "__main__":
    rospy.init_node("wander_bot_node", argv=sys.argv)
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

    my_wander_bot = Wander_bot(forward_speed,rotation_speed, min_scan_angle, max_scan_angle, min_dist_from_obstacle)
    my_wander_bot.start_moving()
