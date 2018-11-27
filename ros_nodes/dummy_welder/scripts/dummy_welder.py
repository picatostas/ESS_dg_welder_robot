#!/usr/bin/env python

import sys, time
import roslib 
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import re

line_regex = re.compile('(\-?\d+\.\d+)') 

class welder:
	def __init__(self):
		self.traj = []
		self.blade_loc_sub = rospy.Subscriber('/blades_location', String, self.traj_callback,queue_size = 16 )
		self.laser_pub = rospy.Publisher('/laser_ctrl/cmd', String, queue_size = 10)
		self.cnc_pub = rospy.Publisher('/cnc_cmd', Twist, queue_size = 10)
		self.grid_ref = [262.5 , 350, 30]

	def traj_callback(self,ros_data):
		point = make_twist(self.grid_ref[0],self.grid_ref[1],self.grid_ref[2])
		self.cnc_pub.publish(point)
		msg = ros_data.data
		#print("line Recieved")
		self.traj.append(line_regex.findall(msg))
		if len(self.traj) == 16:
			print("Received traj")
			traj = self.traj
			self.traj = []
			self.send_traj(traj)

	def send_traj(self, traj):

		#print(traj)
		for blade in range(len(traj)):
			start = self.blade_point(float(traj[blade][0]) ,float(traj[blade][1]), self.grid_ref[2])
			end   = self.blade_point(float(traj[blade][2]) ,float(traj[blade][3]), self.grid_ref[2])
			#self.laser_pub.publish('f')
			#self.cnc_pub.publish(start)
			#self.laser_pub.publish('f')
			#self.cnc_pub.publish(end)
			#self.laser_pub.publish('s')
		print("End of traj")

			#for point in range(len(traj[0])):
			#print("Line number : " + str(blade))
			#print("Line Start : x " + str(traj[blade][0]) +" y " + str(traj[blade][1]))
			#print("Line End : x " + str(traj[blade][2]) +" y " + str(traj[blade][3]))
	def blade_point(self,x,y,z):
		point = Twist()
		point.linear.x  = self.grid_ref[0] 
		point.linear.y  = self.grid_ref[1] 
		point.linear.z  = z 
		point.angular.x = 0
		point.angular.y = 0
		point.angular.z = 0
	
		return point



def main(args):
	dummy_welder = welder()

	rospy.init_node('dummy_welder', anonymous = False)
	rospy.spin()
#main

def make_twist(x,y,z):
	point = Twist()
	point.linear.x  = x 
	point.linear.y  = y 
	point.linear.z  = z 
	point.angular.x = 0
	point.angular.y = 0
	point.angular.z = 0

	return point



if __name__ == '__main__':
    main(sys.argv)

