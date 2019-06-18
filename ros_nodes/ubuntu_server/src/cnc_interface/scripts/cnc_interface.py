#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from xyz import XYZ
acceleration = 0  
x_max 		 = 0  
y_max 		 = 0   
x_max 		 = 0   
x_max_speed  = 0
y_max_speed  = 0
z_max_speed  = 0
x_steps_mm   = 0
y_steps_mm   = 0
z_steps_mm   = 0

xyz = XYZ()


def cmd_move_callback(msg):

	rospy.loginfo(rospy.get_name() + ": " + str(msg))
	print msg.linear.x, msg.linear.y, msg.linear.z
	xyz.moveTo(msg.linear.x, msg.linear.y, msg.linear.z, blockUntilComplete=True)

def stop_callback(msg):
	if   msg.data == 's':
		xyz.disableSteppers()
	elif msg.data == 'f':
		xyz.enableSteppers()

def main():

	pos_pub     = rospy.Publisher('/cnc_interface/position',  Twist, queue_size = 10)
	status_pub  = rospy.Publisher('/cnc_interface/status'  , String, queue_size = 10)

	rospy.init_node('cnc_interface', anonymous=True)
	acc         = rospy.get_param(acceleration)  
	max_x 		= rospy.get_param(x_max)   
	max_y 		= rospy.get_param(y_max)   
	max_z 		= rospy.get_param(x_max)   
	speed_x  	= rospy.get_param(x_max_speed)
	speed_y  	= rospy.get_param(y_max_speed)
	speed_z  	= rospy.get_param(z_max_speed)
	steps_x 	= rospy.get_param(x_steps_mm)
	steps_y 	= rospy.get_param(y_steps_mm)
	steps_z 	= rospy.get_param(z_steps_mm)

	xyz.startup(acc,max_x,max_y,max_z,speed_x,speed_y,speed_z,steps_x,steps_y,steps_z)
	rospy.Subscriber('cnc_interface/cmd' ,  Twist, cmd_move_callback)
	rospy.Subscriber('cnc_interface/stop', String,     stop_callback)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		status     = xyz.getStatus()
		cnc_pose   = xyz.get_pos_Twist()
		ros_status = String(status)
		pos_pub.publish(cnc_pose)
		status_pub.publish(ros_status)
		rate.sleep()

	rospy.spin()

main()
