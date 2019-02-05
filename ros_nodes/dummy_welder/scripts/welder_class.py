#!/usr/bin/env python
import sys, time
import roslib 
import rospy
import numpy as np
from   std_msgs.msg        import String
from   geometry_msgs.msg   import Twist
from   rospy.numpy_msg     import numpy_msg
from   rospy_tutorials.msg import Floats
import re


line_regex = re.compile('(\-?\d+\.\d+)') 


class welder:
	def __init__(self):
		# subscriber handlers
		self.blade_loc_sub          = rospy.Subscriber('/blades_location'       ,String,         self.traj_clbk, queue_size  = 100,
																							    buff_size = 400)#, tcp_nodelay = True)
		self.cnc_pos_sub            = rospy.Subscriber('/cnc_interface/position', Twist,      self.cnc_pos_clbk, queue_size  = 10)
		self.laser_status_sub       = rospy.Subscriber('/laser_ctrl/status'     ,String, self.laser_status_clbk, queue_size  = 10)
		self.start_sub              = rospy.Subscriber('/welder/start'          ,String,        self.start_clbk, queue_size  = 10)
		# publisher handlers
		self.laser_pub              = rospy.Publisher('/laser_ctrl/cmd', String, queue_size = 10)
		self.cnc_pub                = rospy.Publisher('/cnc_cmd',         Twist, queue_size = 10)
		self.line_query_pub         = rospy.Publisher('/detection_query',String, queue_size = 10)
		self.GRIDS_TO_PROCESS 		= 6
		self.LINES_PER_BLADE		= 16
		# reference points Template
		self.grid_refs      = [[445.20, 161.30, 30.00], # [0]  center point 0   [2]    
						       [445.20, 201.30, 30.00], # [1]  center point 0   [1]
						       [445.20, 241.30, 30.00], # [2]  center point 0   [0]
						       [445.20, 281.30, 30.00], # [3]  center point 1   [2]
						       [445.20, 321.30, 30.00], # [4]  center point 1   [1]
						       [445.20, 361.30, 30.00], # [5]  center point 1/2 [0][2]
						       [445.20, 401.30, 30.00], # [6]  center point 2   [1]
						       [445.20, 441.30, 30.00], # [7]  center point 2   [2]
						       [198.00, 439.90, 30.00], # [8]  center point 3   [0]
						       [197.80, 399.90, 30.00], # [9]  center point 3   [1]
						       [197.80, 359.90, 30.00], # [10] center point 3   [2]
						       [197.80, 320.30, 30.00], # [11] center point 4   [0]
						       [197.80, 280.30, 30.00], # [12] center point 4   [1]
						       [197.60, 240.30, 30.00], # [13] center point 4/5 [2][0]
						       [197.60, 200.30, 30.00], # [14] center point 5   [1]
							   [197.60, 159.90, 30.00]] # [15] center point 5   [2]

		self.center_points  = [[362.00, 210.70, 30.00],
							   [362.00, 330.70, 30.00],
							   [362.00, 410.70, 30.00],
							   [115.00, 408.70, 30.00],
							   [115.00, 288.70, 30.00],
							   [115.00, 208.70, 30.00]]	
		#timestamp variables
		self.start_time 	= 0 
		self.detect_time	= 0
		self.detect_time_en	= True 
		self.welding_time	= 0

		self.cnc_pos        = [0.0, 0.0, 0.0]
		# boolean variables for fsm
		self.start          = False
		self.next_is_blade  = False
		self.laser_status   = False
		self.laser_cmd      = False
		self.laser_sync     = True
		# traj related variables
		self.last_point     = [0.0, 0.0, 0.0]
		self.received_traj  = []
		self.traj_completed = False
		self.traj_received  = False
		self.grid_count	    = 0
		self.weld_idx		= 0 
		self.detect_idx		= 0
		self.traj 			= []

	def laser_status_clbk(self,ros_data):

		msg = ros_data.data
		if   msg == '1':#fire
			self.laser_status =  True
		elif msg == '0':#stop
			self.laser_status = False		

	def start_clbk(self,ros_data):

		char = ros_data.data
		if char == 's':
			self.start 		    = True
			self.detect_idx     = 0
			self.detect_time_en = True
			self.traj_completed = False
			self.traj_received  = False

	def cnc_pos_clbk(self,ros_data):

		pos = ros_data
		self.cnc_pos[0] = pos.linear.x
		self.cnc_pos[1] = pos.linear.y
		self.cnc_pos[2] = pos.linear.z

	def traj_clbk(self,ros_data):

		msg    = ros_data.data
		points = []
		grid   = []
		parsed_msg = line_regex.findall(msg)
		points.append(parsed_msg[:2])
		points.append(parsed_msg[2:])
		self.received_traj.append(points)
		if len(self.received_traj) >= 48: #16 blades 3 times
			grids = np.reshape(np.array(self.received_traj),(3,16,2,2))
			np.array(grids).tolist()

			for grid_idx, grid in enumerate(grids):
				if not ((self.detect_idx == 2 and grid_idx == 2) or (self.detect_idx == 5 and grid_idx == 0)):
					self.traj.append(np.array(grid).tolist())
				else:
					print("Already stored grid, skip")

			self.received_traj = []

			print("Grids received: %d , detection idx: %d "%(len(self.traj),self.detect_idx)) 
			print("Overall time: %.2f s , detection time: %.2f s"%((time.time() - self.start_time),
																   (time.time() - self.detect_time)))
			self.detect_idx += 1
			self.traj_received = True
			if self.detect_idx == self.GRIDS_TO_PROCESS:
				for grid_idx, grid in enumerate(self.traj): 
					for blade in grid:
						for point in blade:
							point[0] = float(('%.2f'% (float(point[0]) + self.grid_refs[grid_idx][0])))
							point[1] = float(('%.2f'% (float(point[1]) + self.grid_refs[grid_idx][1])))
							point.append(self.grid_refs[self.detect_idx][2])
				print("traj of %d grids %d blades %d points"%(len(self.traj),
															  len(self.traj[0]),
															  len(self.traj[0][0])))
				#print self.traj[0]
				#print self.traj[0][0]
				#print self.traj[0][0][0]		

	def make_twist(self,p):

		point = Twist()
		point.linear.x  = p[0]
		point.linear.y  = p[1]
		point.linear.z  = p[2]
		point.angular.x = 0.0
		point.angular.y = 0.0
		point.angular.z = 0.0
	
		return point

	def move_to(self, point):

		next_p =  self.make_twist(point)
		print('ROBOT_FSM: Moving to X: %.2f Y: %.2f Z: %.2f' %((next_p.linear.x), (next_p.linear.y), (next_p.linear.z)))
		self.cnc_pub.publish(next_p)

