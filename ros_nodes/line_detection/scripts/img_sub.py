#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
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
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

#Houglines detection
minLineLength = 30
maxLineGap = 50
#canny detection variables
lowTh = 100
HighTh = 300
slope_th = 0.044

#HSV green mask parameters
sensitivity = 20
lower_green = np.array([60 - sensitivity,10,10])
upper_green = np.array([60 + sensitivity,170,170])
# initialize the camera and grab a reference to the raw camera capture
res_v = 720
res_h = 540


VERBOSE=False

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/lines_detected/compressed",
            CompressedImage, queue_size = 10)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /raspicam_node/image/compressed"


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv.imdecode(np_arr, cv.IMREAD_COLOR)        
        h = np.size(image,0)
        w = np.size(image,1)
        #newcamera, roi = cv.getOptimalNewCameraMatrix(K, d, (w,h), 0)
        #image = cv.undistort(image, K, d, None, newcamera)
        image_hsv = cv.cvtColor(image,cv.COLOR_BGR2HSV)
        green_mask = cv.inRange(image_hsv,lower_green,upper_green)
        gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        # canny detection
        edges   = cv.Canny(gray, lowTh, HighTh,apertureSize = 3)
        lines = cv.HoughLinesP(edges,1,np.pi/180,50,None,minLineLength,maxLineGap)
        h_lines = []
        v_lines = []
        if lines is not None:
            for i in range(0,len(lines)):
                
                x1 = lines[i][0][0]
                y1 = lines[i][0][1]
                x2 = lines[i][0][2]
                y2 = lines[i][0][3]
                line_length = length(x1,x2,y1,y2)           
                slope = slope_cal(x1,x2,y1,y2)
                if slope is 0 and line_length < 50:
                    cv.line(image, (x1, y1), (x2, y2), (0, 255, 255), 1, cv.LINE_AA)
                    h_lines.append(lines[i])
                if slope is 1 and line_length < 100:
                    cv.line(image, (x1, y1), (x2, y2), (0, 255, 0), 1, cv.LINE_AA)
                    v_lines.append(lines[i])


        ### Show image 
        cv.imshow('cv_img', image)
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