#!/usr/bin/env python

#imports
import roslib
roslib.load_manifest('coordinate_generator')
import rospy
import math
from geometry_msgs.msg import Twist
import tf
import sys
import pcl

def cnc_callback(ros_data):
	br = tf.TransformBroadcaster()
	x = ros_data.linear.x
	y = ros_data.linear.y
	z = ros_data.linear.z
	#br.sendTransform((,,),(0,0,0,1),rospy.Time.now(),"blades","camera") 
	br.sendTransform((x,y,z),(0,0,0,1),rospy.Time.now(),"camera","cnc")
	br.sendTransform((132,0,186),(0,0,0,1),rospy.Time.now(),"cnc","world")
	

def main(args):
	cnc_sub = rospy.Subscriber('/cnc_interface/position', Twist, cnc_callback)
	cnc_cmd = rospy.Publisher('/cnc_cmd_mock', Twist, queue_size = 10)
	rospy.init_node('coordinate_gen', anonymous = False)

	rospy.spin()
#main

if __name__ == '__main__':
    main(sys.argv)