#!/usr/bin/env python

#imports
import roslib
roslib.load_manifest('coordinate_generator')
import rospy
import math
import tf
import geometry_msgs.msg
#vars

#defs
class coord_gen(object):
	"""docstring for coord_gen"""
	def __init__(self, arg):
		#
		self.cnc_pub = rospy.Publisher('cnc_cmd', Twist, cmd_move_callback,queue_size = 10)
		self.detection_suscribe = rospy.Sus
	def frame_callback(self, ros_data):
		#


def main(args):
	rospy.init_node('coordinate_gen', anonymous = False)



	rate = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		try:
			
		except (tf.Exception,tf.LookupException, tf.ConnectivityException):
		continue

		rate.sleep()
#main

if __name__ == '__main__':
    main(sys.argv)