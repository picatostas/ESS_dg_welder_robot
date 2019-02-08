#!/usr/bin/env python

# Python libs
import sys, time
import glob

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2 as cv

# Ros libraries
import roslib
import rospy
import imutils
import matplotlib.pyplot as plt
# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
#Houglines detection
minLineLength = 10
maxLineGap = 30
#canny detection variables
lowTh = 50
HighTh = 200
len_th = [60,75]
dst_th = [30,60]
slope_th = 0.30 #rads
px_to_mm = 0.1499 # pseudo empiric value
# Distortion parameters 
font = cv.FONT_HERSHEY_SIMPLEX
chunk_factor = 3
image_chunks = []

class image_feature:

    def __init__(self):
        # topic where we publish
        self.ready = True
        self.camera_matrix = 0 
        self.dist_coeff    = 0 
        self.image_pub = rospy.Publisher("/lines_detected/compressed",
            CompressedImage, queue_size = 1)

        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.frame_callback,  queue_size = 10)

        self.info_sub = rospy.Subscriber("/raspicam_node/camera_info",
            CameraInfo, self.info_callback,  queue_size = 1)  

        self.template = cv.imread('/home/multigrid/catkin_ws/src/line_detection/scripts/pattern.png')
        self.template = cv.cvtColor(self.template, cv.COLOR_BGR2GRAY)
        self.template = cv.Canny(self.template, lowTh, HighTh)
        self.han_logo = cv.imread('/home/multigrid/catkin_ws/src/line_detection/scripts/han100.png')
        self.ess_logo = cv.imread('/home/multigrid/catkin_ws/src/line_detection/scripts/ess100.png')
        
    def info_callback(self, ros_data):
           self.camera_matrix = np.reshape(ros_data.K,(3,3))
           self.dist_coeff = ros_data.D
           print("Camera Matrix:\n" + str(self.camera_matrix))
           print("Distortion Coeffs:\n" + str(self.dist_coeff))       
           self.info_sub.unregister()


    def frame_callback(self, ros_data):

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        ### Camera distortion correction
        h = np.size(image,0)
        w = np.size(image,1)
        K = self.camera_matrix
        d = self.dist_coeff
        newcamera, roi = cv.getOptimalNewCameraMatrix(K,d,(w,h),0)
        #image = cv.undistort(image, K,d, None, newcamera)
        gray  = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        canny = cv.Canny(gray, lowTh, HighTh,apertureSize = 3)
        image_chunks = np.vsplit(gray,chunk_factor)
        chunk_size = h / chunk_factor
        for idx, chunk in enumerate(image_chunks):

            
            edges = cv.Canny(chunk, lowTh, HighTh,apertureSize = 3)
            lines = cv.HoughLinesP(edges,1,np.pi/180,40,None,minLineLength,maxLineGap)
            ctr = match_template(self.template, chunk)

            valid_lines = []
            if lines is not None:
                valid_lines = get_lines(ctr,lines)

            cv.circle(image,(ctr[0],ctr[1] + chunk_size*idx), 20, (0,255,0),1,cv.LINE_AA)
            cv.drawMarker(image,(ctr[0],ctr[1] +chunk_size*idx),(0,255,0),cv.MARKER_CROSS ,40,1,cv.LINE_AA)

            for line in valid_lines:
                cv.line(image, (line[0],line[1] + chunk_size*idx), (line[2],line[3] + chunk_size*idx), (200, 0, 200), 2, cv.LINE_AA)

            if idx != len(image_chunks):
                cv.line(image, (0, chunk_size*idx), (w, chunk_size*idx), (0, 255, 255), 1, cv.LINE_AA)

        #cv.imshow('canny',canny)
        #cv.waitKey(2)


        ### IMAGE HEADER 
        han_logo_h = np.size(self.han_logo,0)
        han_logo_w = np.size(self.han_logo,1)
        ess_logo_h = np.size(self.ess_logo,0)
        ess_logo_w = np.size(self.ess_logo,1)
        if han_logo_h > ess_logo_h:
            image[0:han_logo_h,0:w] = [255,255,255]
        else:
            image[0:ess_logo_h,0:w] = [255,255,255]

        image[0:han_logo_h,int(0.72*w):int(0.72*w + han_logo_w)] = self.han_logo
        image[0:ess_logo_h,int(0.15*w):int(0.15*w + ess_logo_w)] = self.ess_logo
        cv.putText(image,str("MG Welder Robot : Han Solo"),(450,40), font, 1,(0,0,0),3,cv.LINE_AA)
        ### IMAGE HEADER

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv.imencode('.jpg', image)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)


def main(args):

    ic = image_feature()
    rospy.init_node('line_detection', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv.destroyAllWindows()


def match_template(template, gray):

    (tH, tW) = template.shape[:2]
    found = None

    for scale in np.linspace(0.2, 1.0, 20)[::-1]:
        # resize the image according to the scale, and keep track
        # of the ratio of the resizing
        resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
        r = gray.shape[1] / float(resized.shape[1]) 
        # if the resized image is smaller than the template, then break
        # from the loop
        if resized.shape[0] < tH or resized.shape[1] < tW:
            break
        edged = cv.Canny(resized, lowTh, HighTh)
        result = cv.matchTemplate(edged, template, cv.TM_CCOEFF)
        (_, maxVal, _, maxLoc) = cv.minMaxLoc(result) 
        # if we have found a new maximum correlation value, then update
        # the bookkeeping variable
        #print(found)
        if found is None or maxVal > found[0]:
            found = (maxVal, maxLoc, r) 
    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
    return [(startX + endX)/2 , (startY + endY)/2]


def get_lines(ctr,lines):
    valid_lines = []
    for element in lines:
        line = element[0]
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        line_length = length(x1,x2,y1,y2)
        slope = slope_cal(x1,x2,y1,y2)        
        # Check if the line is vertical and with the desired length
        #if line_length > 60 and line_length < 75:
        if slope == 1 and line_length > len_th[0] and line_length < len_th[1]:
            # discard the lines that dont start where the blade should start
            if ((ctr[1] - y1) > dst_th[0] and (ctr[1] - y1) < dst_th[1] ) or ((ctr[1] - y2) > dst_th[0] and (ctr[1] - y2) < dst_th[1] ):
                valid_lines.append(line)

    return valid_lines


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