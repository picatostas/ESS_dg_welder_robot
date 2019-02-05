import serial
import time
import re
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# motor driver for XYZ hardware
# Alex & Frank's gantry has the following convention:
# - X axis is perpendicular to the cross bar
# - Z axis is perpendicular to the floor
class XYZ:
	__default_speed__ = 1000	# mm per second
	__pos_pattern__ = re.compile('.Pos:(\-?\d+\.\d+),(\-?\d+\.\d+),(\-?\d+\.\d+)')	# for parsing GRBL feedback
	
	def __init__(self):
		self.cncStateTopic = rospy.Publisher('/cnc_interface/state', String, queue_size = 10)
		self.s = None				# serial port object
		self.abs_move = None		# GRBL has 2 movement modes, relative and absolute
		self.defaultSpeed = 1000	# mm per min
		self.baudrate = 115200
		self.port = '/dev/ttyACM0'
		self.wait_resp = False
		self.acceleration = 1000
		self.x_max = 500
		self.y_max = 500
		self.z_max = 100
		self.x_max_speed = 2000
		self.y_max_speed = 2000
		self.z_max_speed = 2000
		self.x_steps_mm = 2560
		self.y_steps_mm = 2560
		self.z_steps_mm = 5000
		self.idle = True	
		# vectors follow the format [X, Y, Z] where Z is assumed to be vertical
		# number of steps per centimeter in each dimension
		self.pos = [0.0, 0.0, 0.0]		   # current position
		self.angular = [0.0, 0.0, 0.0]
		self.origin = [0.0, 0.0, 0.0]		# minimum coordinates
		self.limits = [self.x_max, self.y_max, self.z_max]	 # maximum coordinates
	
	def startup(self, acc, maxx, maxy,maxz,spdx, spdy, spdz, stepsx, stepsy, stepsz):
		""" initiate connection to the microcontroller """
		#self.baudrate = baud
		#self.port = port
		self.acceleration = acc
		self.x_max = maxx
		self.y_max = maxy
		self.z_max = maxz
		self.x_max_speed = spdx
		self.y_max_speed = spdy
		self.z_max_speed = spdz
		self.x_steps_mm = stepsx
		self.y_steps_mm = stepsy
		self.z_steps_mm = stepsz
		self.s = serial.Serial(self.port, self.baudrate)
		#self.s = serial.Serial('/dev/ttyACM0',115200)

		#time.sleep(2)

		self.ensureMovementMode(True)
		self.home()
		self.set_origin()			# set the current position as the origin (GRBL sometimes starts with z not 0)

	def shutdown(self):
		self.s.close()
		
	def position(self):
		""" return a list [x,y,z] of the position of the gantry head """
		return list(self.pos)	# copy the list so caller can't modify our internal state
	
	def get_pos_Twist(self):

		cnc_pose = Twist()
		cnc_pose.linear.x  = float(self.pos[0])
		cnc_pose.linear.y  = float(self.pos[1])
		cnc_pose.linear.z  = float(self.pos[2])
		cnc_pose.angular.x = float(0)
		cnc_pose.angular.y = float(0)
		cnc_pose.angular.z = float(0)

		return cnc_pose

	def setSpeed(self, speed):
		self.defaultSpeed = speed
		
	def home(self):
		""" return to home position, erasing any drift that has accumulated """
		self.s.write("$H\n")
		self.s.readline()
		self.pos = list(self.origin)
	
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
					#error
					#print self.limits[i] +'=' + pos[i] + ' position outside limit: \n'
					return
				gcode += ' ' + letters[i] + str(pos[i])
				newpos[i] = pos[i]

		gcode += ' F' + str(speed)
		gcode += '\n'
		
		self.s.write(gcode)
		self.s.readline()
		
		#update position if success
		# TODO check to make sure it's actually a success
		#self.pos = newpos
		
		#if blockUntilComplete:
		#	self.blockUntilIdle()

	def moveRel(self, dx=None, dy=None, dz=None, speed=__default_speed__, blockUntilComplete=True):
		""" move a given distance, and return when movement completes
		:param dx, dy, dz: distance to move
		:param speed: units uncertain
		:param blockUntilComplete: whether to return immediately, or wait for the movement to complete
		"""

		self.ensureMovementMode(absoluteMode = False)

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

	def moveToOrigin(self, speed=__default_speed__):
		""" move to starting position, and return when movement completes """
		self.moveTo(*self.origin, speed=speed)
		self.pos = list(self.origin)
			
	def set_origin(self, x=0, y=0, z=0):
		"""set current position to be (0,0,0), or a custom (x,y,z)"""
		gcode = "G92 x{} y{} z{}\n".format(x, y, z)
		self.s.write(gcode)
		self.s.readline()
		
		# update our internal location
		self.pos = [x, y, z]

	def ensureMovementMode(self, absoluteMode = True):
		""" GRBL has two movement modes; if necessary this function tells GRBL to switch modes """
		# nothing to do?
		if self.abs_move == absoluteMode: return
		
		self.abs_move = absoluteMode
		if absoluteMode:
			self.s.write("G90\n")		# absolute movement mode
		else:
			self.s.write("G91\n")		# relative movement mode
		self.s.readline()
	
	# deprecated by extending getStatus functionallity 	
	def blockUntilIdle(self):
		""" polls until GRBL indicates it is done with the last command """
		pollcount = 0
		while True:
			self.s.write("?")
			status = self.s.readline()
			if status.startswith('<Idle'): break
			pollcount += 1
			time.sleep(.01)		# poll every 10 ms

		
	def getStatus(self):

		self.s.write("?")
		
		while True:
			status = self.s.readline()
			if status is not None:
				try:
					matches = self.__pos_pattern__.findall(status)
					#if matches is not None:
					if len(matches[1]) == 3:
						self.pos = list(matches[1])				
					return status
				except IndexError:
					print("No matches found in serial")
			else: break