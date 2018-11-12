#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2 as cv

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo

#Houglines detection
minLineLength = 10
maxLineGap = 30
#canny detection variables
lowTh = 50
HighTh = 200
slope_th = 0.044 #rads

#HSV green mask parameters
sensitivity = 20
lower_green = np.array([60 - sensitivity,0,0])
upper_green = np.array([60 + sensitivity,200,200])

# Distortion parameters 

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.VERBOSE = False
        self.ready = True
        #### valores para 720x480
        #self.camera_matrix = np.array([[620.5487720046755, 0.0, 375.5880513383576],[0.0, 582.7663413137564, 239.32314467548184],[0.0, 0.0, 1.0]])
        #self.dist_coeff = np.array([0.20348128827432369, -0.12919690343587154, -0.01216483341116271, 0.019875488308653443,-0.6471023360662196])

        #### valores para 1280x720
        self.camera_matrix = np.array([[1276.704618338571, 0, 634.8876509199106],[0, 1274.342831275509, 379.8318028940378],[0.0, 0.0, 1.0]])
        self.dist_coeff    = np.array([0.1465167016954302, -0.2847343180128725, 0.00134017721235817, -0.004309553450829512, 0])
        
        self.image_pub = rospy.Publisher("/lines_detected/compressed",
            CompressedImage, queue_size = 1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.frame_callback,  queue_size = 10)

        #self.subs = rospy.Subscriber("/raspicam_node/camera_info",
        #    CameraInfo, self.info_callback,  queue_size = 1)       
        
        if self.VERBOSE :
            print "subscribed to /raspicam_node/image/compressed"


    #def info_callback(self, ros_data):
    #	if self.ready :
	#		self.camera_matrix = ros_data.K
	#		self.dist_coeff    = ros_data.D
	#		print("received camera info with contents ->")
	#		print("Camera Matrix :\n" + str(self.camera_matrix))
	#		print("Distortion Coefficients :\n" + str(self.dist_coeff))
	#		self.ready = False

    def frame_callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if self.VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        #raw_img = image

        h = np.size(image,0)
        w = np.size(image,1)
        K = self.camera_matrix
        d = self.dist_coeff

        newcamera, roi = cv.getOptimalNewCameraMatrix(K,d,(w,h),0)
        image = cv.undistort(image, K,d, None, newcamera)
        #undistored = image
        image_hsv = cv.cvtColor(image,cv.COLOR_BGR2HSV)
        green_mask = cv.inRange(image_hsv,lower_green,upper_green)
        gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        # canny detection
        edges   = cv.Canny(gray, lowTh, HighTh,apertureSize = 3)
        edges_mask = cv.Canny(green_mask, lowTh, HighTh, apertureSize = 7)

        lines 	   = cv.HoughLinesP(edges,1,np.pi/180,50,None,minLineLength,maxLineGap)
        lines_mask = cv.HoughLinesP(edges_mask,1,np.pi/180,10,None,30,maxLineGap)
        image = draw_lines(lines,image,0,0,255,2)
                  
        ### Show image 
        #cv.imshow('image',image)
    	#cv.imshow('canny',edges)
    	#cv.imshow('undistored', undistored)
    	#cv.imshow('distorted',raw_img)
    	#cv.imshow('green mask', image_mask)
    	cv.waitKey(2)


        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv.imencode('.jpg', image)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv.destroyAllWindows()
def draw_lines(lines, img,r = 255,g = 0,b = 0, thickness = 1):
    if lines is not None:
        for i in range(0,len(lines)):
        	#h_lines = []
        	#v_lines = []
            x1 = lines[i][0][0]
            y1 = lines[i][0][1]
            x2 = lines[i][0][2]
            y2 = lines[i][0][3]
            line_length = length(x1,x2,y1,y2)           
            slope = slope_cal(x1,x2,y1,y2)
            if slope == 1 and line_length > 50 and line_length < 90:
                cv.line(img, (x1, y1), (x2, y2), (b, g, r), thickness, cv.LINE_AA)
            #if slope is 0 and line_length < 50:
            #    cv.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2, cv.LINE_AA)
            #    h_lines.append(lines[i])
            #if slope is 1 and line_length < 100:
            #    cv.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2, cv.LINE_AA)
            #    v_lines.append(lines[i])
            #elif line_length < 400:
            #    cv.line(image, (x1, y1), (x2, y2), (255, 0, 255), 2, cv.LINE_AA)
            #    v_lines.append(lines[i])

    return img
def slope_cal(x0,x1,y0,y1):
    slope = np.arctan2((y1-y0),(x1-x0))
    if  np.abs(slope - 0) < slope_th or np.abs(slope - np.pi) < slope_th:
        return 0
    if np.abs(slope - np.pi/2) < slope_th or np.abs(slope + np.pi/2) < slope_th:
        return 1
    return 2


def length(x0,x1,y0,y1):
    return np.sqrt(np.power(y1-y0,2) + np.power(x1-x0,2))



if __name__ == '__main__':
    main(sys.argv)