import serial
import time
import re
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

''' Class valid for interfacing XYZ cartesian CNC
	Future implementations might include control for other 
	GCODE compatible systems
'''

class cnc:
	# regular expression for parsing GRBL status msgs
	__pos_pattern__ = re.compile('.Pos:(\-?\d+\.\d+),(\-?\d+\.\d+),(\-?\d+\.\d+)')	
	
	def __init__(self):
		self.s 		      =   None	# serial port object
		self.abs_move     =   None	# GRBL has 2 movement modes, relative and absolute
		# Default parameter values set in startup
		self.baudrate 	  = 	 0
		self.port 		  =     ''
		self.acceleration =   	 0
		self.x_max        =   	 0
		self.y_max        =   	 0
		self.z_max        =   	 0
		self.defaultSpeed =      0	
		self.x_max_speed  =   	 0
		self.y_max_speed  =   	 0
		self.z_max_speed  =   	 0
		# number of steps per centimeter in each dimension
		self.x_steps_mm   =   	 0
		self.y_steps_mm   =   	 0
		self.z_steps_mm   =   	 0
		# machine idle
		self.idle 		  =   True	
		# vectors follow the format [X, Y, Z] where Z is assumed to be vertical
		self.pos     = [0.0, 0.0, 0.0]   # current position
		self.angular = [0.0, 0.0, 0.0]	 # angular coordinates
		self.origin  = [0.0, 0.0, 0.0]	 # minimum coordinates
		self.limits  = [0.0, 0.0, 0.0]	 # maximum coordinates
	
	def startup(self,port,baud, acc, maxx, maxy,maxz,spdf,spdx, spdy, spdz, stepsx, stepsy, stepsz):
		""" initiate all CNC parameters readed from .launch file """
		self.baudrate 	  =   baud
		self.port 	      =   port
		self.acceleration =    acc
		self.x_max        =   maxx
		self.y_max        =   maxy
		self.z_max        =   maxz
		self.defaultSpeed =   spdf
		self.x_max_speed  =   spdx
		self.y_max_speed  =   spdy
		self.z_max_speed  =   spdz
		self.x_steps_mm   = stepsx
		self.y_steps_mm   = stepsy
		self.z_steps_mm   = stepsz
		self.limits  = [self.x_max, self.y_max, self.z_max]	
		#initiates the serial port
		self.s = serial.Serial(self.port, self.baudrate)
		# set movement to Absolut coordinates
		self.ensureMovementMode(True)
		# start homing procedure
		self.home()
		# set the current position as the origin (GRBL sometimes starts with z not 0)
		self.setOrigin()	

	def shutdown(self):
		# close the serial connection
		self.s.close()
		
	def getPos(self):
		""" return a list [x,y,z] of the position of the gantry head """
		return list(self.pos)	# copy the list so caller can't modify our internal state
	
	def getTwist(self):

		#convert coordinates to ROS Twist format to be able to publish it later
		cnc_pose = Twist()
		cnc_pose.linear.x  = float(self.pos[0])
		cnc_pose.linear.y  = float(self.pos[1])
		cnc_pose.linear.z  = float(self.pos[2])
		# this parameters are set to 0 as the cnc its a XYZ 3 DOF mechanism and doesnt need it
		cnc_pose.angular.x = float(self.angular[0])
		cnc_pose.angular.y = float(self.angular[1])
		cnc_pose.angular.z = float(self.angular[2])

		return cnc_pose

	def setSpeed(self, speed):

		self.defaultSpeed = speed
		
	def home(self):
		# initaites the home procedure
		self.s.write("$H\n")
		self.s.readline()
		self.pos = list(self.origin)

	def enableSteppers(self):
		# enable the stepper motors
		try:
			self.s.write("M17\n")
			self.s.readline()
		except:
			print("Serial port unavailable")	

	def disableSteppers(self):
		# Disable the stepper motors
		try:
			self.s.write("M18\n")
			self.s.readline()
		except:
			print("Serial port unavailable")
	
	def moveTo(self, x=None, y=None, z=None, speed=None, blockUntilComplete=True):
		""" move to an absolute position, and return when movement completes """
		if not self.idle: return
		if x is None and y is None and z is None: return
		if speed is None: speed = self.defaultSpeed
		
		self.ensureMovementMode(absoluteMode = True)
		
		gcode = 'G0'
		letters = 'XYZ'
		pos = (x, y, z)
		newpos = list(self.pos)
		
		#create gcode string and update position list for each argument that isn't None
		for i in range(3):
			if pos[i] is not None:
				#check against self.limits
				if pos[i] < 0 or pos[i] >= self.limits[i]:
					# if position is outside the movement range, ignore
					return
				gcode += ' ' + letters[i] + str(pos[i])
				newpos[i] = pos[i]

		gcode += ' F' + str(speed)
		gcode += '\n'
		try:
			self.s.write(gcode)
			self.s.readline()
		except:
			print("Serial port unavailable")

	def moveRel(self, dx=None, dy=None, dz=None, speed=None, blockUntilComplete=True):
		""" move a given distance, and return when movement completes
		:param dx, dy, dz: distance to move
		:param speed: units uncertain
		:param blockUntilComplete: whether to return immediately, or wait for the movement to complete
		"""

		self.ensureMovementMode(absoluteMode = False)
		if speed is None: speed = self.defaultSpeed
		gcode = 'G0'
		letters = 'xyz'
		d = (dx, dy, dz)
		newpos = list(self.pos)
		
		#create gcode string and update position list for each argument that isn't None (TODO: if successful?)
		for i in range(3):
			if d[i] is not None:
				gcode += ' ' + letters[i] + str(d[i])
				newpos[i] += d[i]
		
		gcode += ' f' + str(speed)
		gcode += '\n'
		
		self.s.write(gcode)
		self.s.readline()		

		# the position update should be done after reading state
		#update position if success
		# TODO check to make sure it's actually a success
		#self.pos = newpos
		
		if blockUntilComplete:
			self.blockUntilIdle()

	def moveToOrigin(self, speed = None):
		""" move to starting position, and return when movement completes """
		if speed is None: speed = self.defaultSpeed
		self.moveTo(*self.origin, speed=speed)
		self.pos = list(self.origin)
			
	def setOrigin(self, x=0, y=0, z=0):
		"""set current position to be (0,0,0), or a custom (x,y,z)"""
		gcode = "G92 x{} y{} z{}\n".format(x, y, z)
		self.s.write(gcode)
		self.s.readline()
		
		# update our internal location
		self.pos = [x, y, z]

	def ensureMovementMode(self, absoluteMode = True):
		""" GRBL has two movement modes; if necessary this function tells GRBL to switch modes """
		if self.abs_move == absoluteMode: return
		
		self.abs_move = absoluteMode
		if absoluteMode:
			self.s.write("G90\n")		# absolute movement mode
		else:
			self.s.write("G91\n")		# relative movement mode
		self.s.readline()
	

	def blockUntilIdle(self):
		""" polls until GRBL indicates it is done with the last command """
		pollcount = 0
		while True:
			self.s.write("?")
			status = self.s.readline()
			if status.startswith('<Idle'): break
			# not used
			pollcount += 1
			# poll every 10 ms
			time.sleep(.01)		

		
	def getStatus(self):

		self.s.write("?")
		
		while True:
			try: 
				status = self.s.readline()
				if status is not None:
					try:
						matches = self.__pos_pattern__.findall(status)
						if len(matches[1]) == 3:
							self.pos = list(matches[1])				
						return status
					except IndexError:
						print("No matches found in serial")
				else: break
			except:
				print("Report readiness but empty")