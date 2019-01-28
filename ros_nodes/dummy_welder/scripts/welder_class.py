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
		# subscriber handlers
		self.blade_loc_sub          = rospy.Subscriber('/blades_location', String, self.traj_callback,queue_size = 16 )
		self.cnc_pos_sub            = rospy.Subscriber('/cnc_interface/position', Twist, self.cnc_pos_callback, queue_size = 10)
		self.laser_status_sub       = rospy.Subscriber('/laser_ctrl/status', String,self.laser_status_callback, queue_size = 10)
		self.start_sub              = rospy.Subscriber('/welder/start', String, self.start_callback, queue_size = 10)
		# publisher handlers
		self.laser_pub              = rospy.Publisher('/laser_ctrl/cmd', String, queue_size = 10)
		#self.laser_status_query_pub = rospy.Publisher('/laser_ctrl/laser_status_query', String, queue_size = 10)
		self.cnc_pub                = rospy.Publisher('/cnc_cmd', Twist, queue_size = 10)
		self.line_query_pub         = rospy.Publisher('/detection_query',String, queue_size = 10)
		
		## reference points Template
		#self.grid_ref     = [336.00, 402.70, 30.00]
		#self.center_point = [254.50, 409.00, 30.00]
		##

		## reference points Real Grid
		self.grid_ref     = [340.50, 219.30, 20.00]
		self.center_point = [260.90, 229.10, 20.00]	
		##

		self.cnc_pos      = [   0.0,    0.0,   0.0]
		self.start        = False
		# boolean variables for fsm
		self.next_is_blade  = False
		self.laser_status = False
		self.laser_cmd = False
		self.laser_sync = True
		# traj related variables
		self.last_point = [ 0.0, 0.0, 0.0]
		self.traj = []
		self.received_traj = []
		self.traj_completed = False
		self.traj_received  = False

	def laser_status_callback(self,ros_data):

		msg = ros_data.data
		if   msg == '1':#fire
			self.laser_status = True
		elif msg == '0':#stop
			self.laser_status = False		

	def start_callback(self,ros_data):

		char = ros_data.data
		if char == 's':
			#self.laser_status_query_pub.publish('q')
			self.start = True

	def cnc_pos_callback(self,ros_data):

		pos = ros_data
		self.cnc_pos[0] = pos.linear.x
		self.cnc_pos[1] = pos.linear.y
		self.cnc_pos[2] = pos.linear.z

	def traj_callback(self,ros_data):

		msg = ros_data.data
		points = []
		parsed_msg = line_regex.findall(msg)
		points.append(parsed_msg[:2])
		points.append(parsed_msg[2:])
		self.received_traj.append(points)

		if len(self.received_traj) == 16:
			print("Received traj size of %d blades with %d points, %d coords" %(len(self.received_traj),len(self.received_traj[0]),len(self.received_traj[0][0])))
			for blade in self.received_traj:
				for point in blade:
					point[0] = float(('%.2f'% float(point[0]) )) + self.grid_ref[0]
					point[1] = float(('%.2f'% float(point[1]) )) + self.grid_ref[1]
					point.append(self.grid_ref[2])
			self.traj = self.received_traj
			self.traj_received = True
			self.traj_complete = False
			self.received_traj = []
			#self.print_traj()


	def print_traj(self):

		for blade in self.traj:
			print("Line Start -> X : " + str((float(blade[0][0])) ) + " Y: " + str((float(blade[0][1])) ))#- 0.35 ))
			print("Line End   -> X : " + str((float(blade[1][0])) ) + " Y: " + str((float(blade[1][1])) ))#- 0.35 ))

		print("End of traj")

	def blade_point(self,p):
		point = Twist()
		point.linear.x  = self.grid_ref[0] + float(p[0])
		point.linear.y  = self.grid_ref[1] + float(p[1])
		point.linear.z  = self.grid_ref[2] 
		point.angular.x = 0
		point.angular.y = 0
		point.angular.z = 0
	
		return point

	def make_twist(self,p):
		point = Twist()
		point.linear.x  = p[0]
		point.linear.y  = p[1]
		point.linear.z  = p[2] 
		point.angular.x = 0
		point.angular.y = 0
		point.angular.z = 0
	
		return point

	def move_to(self, point, conv):

		if conv:
			next_p = self.blade_point(point)
		else:
			next_p = self.make_twist(point)

		print('ROBOT_FSM: Moving to X: %.2f Y: %.2f Z: %.2f' %((next_p.linear.x) , (next_p.linear.y) ,(next_p.linear.z)))
		self.cnc_pub.publish(next_p)

