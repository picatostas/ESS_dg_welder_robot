#!/usr/bin/env python

#imports
import roslib
roslib.load_manifest('coordinate_generator')
import rospy
import math
from geometry_msgs.msg import Twist
import tf
import sys
def cnc_callback(ros_data):
	br = tf.TransformBroadcaster()
	x = ros_data.linear.x
	y = ros_data.linear.y
	z = ros_data.linear.z 
	br.sendTransform((x,y,z),(0,0,0,1),rospy.Time.now(),"camera","cnc")
	br.sendTransform((132,0,186),(0,0,0,1),rospy.Time.now(),"cnc","world")


def main(args):
	rospy.init_node('coordinate_gen', anonymous = False)
	cnc_sub = rospy.Subscriber('cnc_position_state', Twist, cnc_callback)

	rospy.spin()
#main

if __name__ == '__main__':
    main(sys.argv)