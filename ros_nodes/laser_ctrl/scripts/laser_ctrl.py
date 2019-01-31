#!/usr/bin/env python

import sys, time
import numpy as np 
import cv2 as cv 
import roslib 
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
font = cv.FONT_HERSHEY_SIMPLEX
img_path = '/home/multigrid/catkin_ws/src/line_detection/scripts/'
class laser_ctrl:

	def __init__(self):
		self.location_pub  = rospy.Publisher("/laser_ctrl/img_feed/compressed", CompressedImage, queue_size = 1)
		self.image_sub     = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.image_callback, queue_size = 10)
		self.laser_status  = rospy.Subscriber("/laser_ctrl/status",String, self.cmd_callback, queue_size = 10)
		self.status        = False
		self.camera_matrix = np.array([[1014.343103379204, 0, 637.2463708126373],[0, 1011.69373183754, 469.5663779911617],[0.0, 0.0, 1.0]])
		self.dist_coeff    = np.array([0.1570058008946036, -0.2862704919204555, -5.60164774255961e-05, 0.001586362091473342, 0])		
		self.han_logo = cv.imread(img_path + 'han100.png')
		self.ess_logo = cv.imread(img_path + 'ess100.png')
		self.han_logo_h = np.size(self.han_logo,0)
		self.han_logo_w = np.size(self.han_logo,1)
		self.ess_logo_h = np.size(self.ess_logo,0)
		self.ess_logo_w = np.size(self.ess_logo,1)  
	def cmd_callback(self,ros_data):
		msg = ros_data.data
		if   msg == '1':#fire
			self.status = True
		elif msg == '0':#stop
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
		if self.han_logo_h > self.ess_logo_h:
		    image[0:self.han_logo_h,0:w] = [255,255,255]
		else:
		    image[0:self.ess_logo_h,0:w] = [255,255,255]    
		if self.status:
			cv.putText(image,str("ON"),(int(0.72*w + self.han_logo_w), self.han_logo_h - 25), font, 3,(0,0,255),3,cv.LINE_AA)
			cv.drawMarker(image,(w/2,h/2),(0,0,255),cv.MARKER_CROSS ,60,2,cv.LINE_AA)
			cv.circle(image,(w/2,h/2), 30,(0,0,255),3,cv.LINE_AA)		
		else:
			cv.putText(image,str("OFF"),(int(0.72*w + self.han_logo_w), self.han_logo_h - 25), font, 3,(0,255,0),3,cv.LINE_AA)
			cv.drawMarker(image,(w/2,h/2),(0,255,0),cv.MARKER_CROSS ,60,2,cv.LINE_AA)
			cv.circle(    image,(w/2,h/2), 30,(0,255,0),3,cv.LINE_AA)
		image[0:self.han_logo_h,int(0.72*w):int(0.72*w + self.han_logo_w)] = self.han_logo
		image[0:self.ess_logo_h,int(0.15*w):int(0.15*w + self.ess_logo_w)] = self.ess_logo
		chunk_size = h /3 
		cv.line(image, (0, chunk_size*1), (w, chunk_size*1), (0, 255, 255), 1, cv.LINE_AA)
		cv.line(image, (0, chunk_size*2), (w, chunk_size*2), (0, 255, 255), 1, cv.LINE_AA)
		cv.putText(image,str("MG Welder Robot : Han Solo"),(420,40), font, 1,(0,0,0),3,cv.LINE_AA)
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv.imencode('.jpg', image)[1]).tostring()
		self.location_pub.publish(msg)

def main(args):
    laser = laser_ctrl()
    rospy.init_node('laser_ctrl', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
