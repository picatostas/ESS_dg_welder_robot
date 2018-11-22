#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from xyz import XYZ
#baudrate = 0   
#port = ''
acceleration = 0  
x_max = 0  
y_max = 0   
x_max = 0   
x_max_speed = 0
y_max_speed = 0
z_max_speed = 0
x_steps_mm = 0
y_steps_mm = 0
z_steps_mm = 0

xyz = XYZ()
angular = [0,0,0]

respPositionTopic = None

def cmd_move_callback(msg):
	rospy.loginfo(rospy.get_name() + ": " + str(msg))
	print msg.linear.x, msg.linear.y, msg.linear.z
	xyz.moveTo(msg.linear.x, msg.linear.y, msg.linear.z, blockUntilComplete=True)

#def query_position_callback(msg):
#	respPositionTopic.publish(Twist(*xyz.pos))

def init_cnc_interface():
	rospy.init_node('cnc_interface', anonymous=True)
	#baud         = rospy.get_param(baudrate)
	#prt          = rospy.get_param(port)  
	acc          = rospy.get_param(acceleration)  
	max_x 		 = rospy.get_param(x_max)   
	max_y 		 = rospy.get_param(y_max)   
	max_z 		 = rospy.get_param(x_max)   
	speed_x  	 = rospy.get_param(x_max_speed)
	speed_y  	 = rospy.get_param(y_max_speed)
	speed_z  	 = rospy.get_param(z_max_speed)
	steps_x 	 = rospy.get_param(x_steps_mm)
	steps_y 	 = rospy.get_param(y_steps_mm)
	steps_z 	 = rospy.get_param(z_steps_mm)
	xyz.startup(acc,max_x,max_y,max_z,speed_x,speed_y,speed_z,steps_x,steps_y,steps_z)

	rospy.Subscriber('cnc_cmd', Twist, cmd_move_callback)
	#rospy.Subscriber('query_position', String, query_position_callback)
	respPositionTopic = rospy.Publisher('cnc_position_state', Twist, queue_size = 10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		cnc_pose = Twist()
		cnc_pose.linear.x = xyz.pos[0]
		cnc_pose.linear.y = xyz.pos[1]
		cnc_pose.linear.z = xyz.pos[2]
		cnc_pose.angular.x = xyz.angular[0]
		cnc_pose.angular.y = xyz.angular[1]
		cnc_pose.angular.z = xyz.angular[2]
		rospy.loginfo(Twist)
		respPositionTopic.publish(cnc_pose)
		rate.sleep()
	rospy.spin()

init_cnc_interface()
