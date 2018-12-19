#!/usr/bin/env python

import sys, time
import numpy as np 
import roslib 
import rospy
from std_msgs.msg import String
from gpiozero import LED

led = LED(18)


class led_ctrl:

	def __init__(self):
		self.cmd_sub           = rospy.Subscriber("/laser_ctrl/cmd", String,self.cmd_callback , queue_size = 10)
		self.status_pub        = rospy.Publisher("/laser_ctrl/status", String, queue_size = 10)
		self.status_query_sub  = rospy.Subscriber("/laser_ctrl/status_query", String, self.status_query_callback, queue_size = 10)
		self.laser_status  = '0'

	def status_query_callback(self, ros_data):

		if ros_data.data == 'q':
			self.status_pub.publish(self.laser_status)

	def cmd_callback(self,ros_data):
		msg = ros_data.data
		print("%c cmd received" %msg)
		if   msg == 'f':#fire
			#turn on the led
			led.on()
			self.laser_status = '1'
			self.status_pub.publish(self.laser_status)
		elif msg == 's':#stop
			#turn off the led
			led.off()
			self.laser_status = '0'
			self.status_pub.publish(self.laser_status)

def main(args):
    led = led_ctrl()
    rospy.init_node('led_ctrl', anonymous=False)
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #led.status_pub.publish(led.laser_status)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
