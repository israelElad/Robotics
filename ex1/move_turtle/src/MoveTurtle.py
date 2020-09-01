#!/usr/bin/python
#
# Move_Turtle.py
#
#  Created on: Nov 9, 2016
#      Author: Elad Israel 313448888
#

import sys, rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

#determines whether location should be displayed or not
shouldDisplay=True

def pose_callback(pose_msg):
    global shouldDisplay
    if shouldDisplay==True:
    	rospy.loginfo("x: %.2f, y: %.2f" % (pose_msg.x, pose_msg.y))
	shouldDisplay=False


if __name__ == "__main__":
    FORWARD_SPEED_MPS = 1
    count=1
    robot_name = sys.argv[1]

    # Initialize the node
    rospy.init_node("move_turtle")

    # A publisher for the movement data
    pub = rospy.Publisher(robot_name+"/cmd_vel", Twist, queue_size=10)

    # A listener for pose
    sub = rospy.Subscriber(robot_name+"/pose", Pose, pose_callback)

    # Drive forward at a given speed.  The robot points up the x-axis.
    # The default constructor will set all commands to 0
    msg = Twist()
    msg.linear.x = FORWARD_SPEED_MPS

    # Loop at 10Hz, publishing movement commands until we shut down
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	#move forward 14 steps
	if count<15:
        	pub.publish(msg)
        	rate.sleep()
	#rotate after moving forward
	elif count==16:
		#dont move forward
		msg.linear.x =0
		msg.angular.z= 0.5
		pub.publish(msg)
        	rate.sleep()
		#display last location
		shouldDisplay=True
	count+=1
