#!/usr/bin/env python

import sys, time
import numpy as np 
import cv2 as cv 
import roslib 
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
font = cv.FONT_HERSHEY_SIMPLEX

class laser_ctrl:

	def __init__(self):
		#self.pos = [0,0,0]
		self.location_pub  = rospy.Publisher("/laser_ctrl/img_feed/compressed", CompressedImage, queue_size = 1)
		self.cmd_sub       = rospy.Subscriber("/laser_ctrl/cmd", String,self.cmd_callback , queue_size = 10)
		self.image_sub     = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.image_callback, queue_size = 10)
		self.status        = False
		self.camera_matrix = np.array([[1014.343103379204, 0, 637.2463708126373],[0, 1011.69373183754, 469.5663779911617],[0.0, 0.0, 1.0]])
		self.dist_coeff    = np.array([0.1570058008946036, -0.2862704919204555, -5.60164774255961e-05, 0.001586362091473342, 0])		

	def cmd_callback(self,ros_data):
		msg = ros_data.data
		if   msg == 'f':#fire
			self.status = True
		elif msg == 's':#stop
			self.status = False

	def image_callback(self,ros_data):
		np_image = np.fromstring(ros_data.data, np.uint8)
		image = cv.imdecode(np_image, cv.IMREAD_COLOR)
		h = np.size(image,0)
		w = np.size(image,1)
		K = self.camera_matrix
		d = self.dist_coeff
		newcamera, roi = cv.getOptimalNewCameraMatrix(K,d,(w,h),0)
		image = cv.undistort(image, K,d, None, newcamera)

		cv.putText(image,str("MG Laser welder"),((w/2 - 100),40), font, 1,(0,0,0),3,cv.LINE_AA)
		cv.putText(image,str("STATUS: "),(10,60), font, 2,(0,0,0),3,cv.LINE_AA)

		if self.status:
			cv.putText(image,str("ON"),(250,70), font, 3,(0,0,255),3,cv.LINE_AA)
			cv.drawMarker(image,(w/2,h/2),(0,0,255),cv.MARKER_CROSS ,40,1,cv.LINE_AA)
			cv.circle(image,(w/2,h/2), 20,(0,0,255),1,cv.LINE_AA)		
		else:
			cv.putText(image,str("OFF"),(250,70), font, 3,(0,255,0),3,cv.LINE_AA)
			cv.drawMarker(image,(w/2,h/2),(0,255,0),cv.MARKER_CROSS ,40,1,cv.LINE_AA)
			cv.circle(image,(w/2,h/2), 20,(0,255,0),1,cv.LINE_AA)
		
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv.imencode('.jpg', image)[1]).tostring()
		self.location_pub.publish(msg)

	#def broadcast_pos(self):
def main(args):
    laser = laser_ctrl()
    rospy.init_node('laser_ctrl', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)