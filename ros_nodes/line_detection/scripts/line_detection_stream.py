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
slope_th = 0.044 #rads
px_to_mm = 0.1499 # pseudo empiric value
#HSV green mask parameters
sensitivity = 20
lower_green = np.array([60 - sensitivity,0,0])
upper_green = np.array([60 + sensitivity,200,200])
# Distortion parameters 
font = cv.FONT_HERSHEY_SIMPLEX
frames = []


class image_feature:

    def __init__(self):
        # topic where we publish
        self.VERBOSE = False
        self.ready = True
        #### values for 1280x720 default
        #self.camera_matrix = np.array([[1276.704618338571, 0, 634.8876509199106],[0, 1274.342831275509, 379.8318028940378],[0.0, 0.0, 1.0]])
        #self.dist_coeff    = np.array([0.1465167016954302, -0.2847343180128725, 0.00134017721235817, -0.004309553450829512, 0])
        #### values for 1280x960 default
        self.camera_matrix = np.array([[1014.343103379204, 0, 637.2463708126373],[0, 1011.69373183754, 469.5663779911617],[0.0, 0.0, 1.0]])
        self.dist_coeff    = np.array([0.1570058008946036, -0.2862704919204555, -5.60164774255961e-05, 0.001586362091473342, 0])
        



        self.image_pub = rospy.Publisher("/lines_detected/compressed",
            CompressedImage, queue_size = 1)

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

        ### Camera distortion correction
        h = np.size(image,0)
        w = np.size(image,1)
        lines_hist = [0]*w
        K = self.camera_matrix
        d = self.dist_coeff
        #print("w: " + str(w) + " h: " + str(h))
        newcamera, roi = cv.getOptimalNewCameraMatrix(K,d,(w,h),0)
        image = cv.undistort(image, K,d, None, newcamera)
        ###

        #### Green Mask 
        #image_hsv = cv.cvtColor(image,cv.COLOR_BGR2HSV)
        #green_mask = cv.inRange(image_hsv,lower_green,upper_green)
        ####

        ### canny detection
        gray  = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        edges = cv.Canny(gray, lowTh, HighTh,apertureSize = 3)
        lines = cv.HoughLinesP(edges,1,np.pi/180,40,None,minLineLength,maxLineGap)
        ###

        #### FINDING CORNERS 
        gray_harris = np.float32(gray)
        dst = cv.cornerHarris(gray_harris,2,3,0.04)
        dst = cv.dilate(dst, None)
        ret, dst = cv.threshold(dst,0.01*dst.max(),255,0)
        dst = np.uint8(dst)
        # find centroids
        ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)
        # define the criteria to stop and refine the corners
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.01)
        corners = cv.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
        # Now draw them
        res = np.hstack((centroids,corners))
        res = np.int0(res)
        image[res[:,3],res[:,2]] = [0,255,0]
        cv.putText(image,str("Corners: " + str(len(corners))),(40,120), font, 1,(0,0,0),2,cv.LINE_AA)
        ####

        ### Pattern recognition 
        template = cv.imread('/home/multigrid/catkin_ws/src/line_detection/scripts/pattern.png')
        startX, startY, endX, endY = match_template(template, image)
        cv.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 1)
        centerX = (startX + endX)/2
        centerY = (startY + endY)/2
        #print("Center of the marker: " + str(centerX) + "," + str(centerY))
        ###

        if lines is not None:
            image, lines = draw_lines(centerX,centerY,lines,lines_hist,image,0,0,255,2)

        cv.putText(image,str("MG Welder Robot"),(550,40), font, 1,(0,0,0),3,cv.LINE_AA)
        cv.drawMarker(image,(centerX,centerY),(0,255,0),cv.MARKER_CROSS ,40,1,cv.LINE_AA)
        cv.drawMarker(image,(w/2,h/2),(0,255,255),cv.MARKER_TILTED_CROSS ,80,1,cv.LINE_AA)
        cv.circle(image,(centerX,centerY), 20, (0,255,0),1,cv.LINE_AA)

        #### Show image 
        #cv.imshow('template',template)
        #cv.imshow('image',res)
    	#cv.imshow('canny',edges)
    	#cv.imshow('undistored', undistored)
    	#cv.imshow('distorted',raw_img)
    	#cv.imshow('green mask', image_mask)
        ####
    	cv.waitKey(2)


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


def match_template(template, image):
    template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    template = cv.Canny(template, 50, 200)
    (tH, tW) = template.shape[:2]
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
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
        edged = cv.Canny(resized, 50, 200)
        result = cv.matchTemplate(edged, template, cv.TM_CCOEFF)
        (_, maxVal, _, maxLoc) = cv.minMaxLoc(result) 
        # if we have found a new maximum correlation value, then update
        # the bookkeeping variable
        if found is None or maxVal > found[0]:
            found = (maxVal, maxLoc, r) 
    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r)) 
    return startX, startY, endX, endY


def draw_lines(centerX,centerY,lines,lines_hist, img,r = 255,g = 0,b = 0, thickness = 1):
    #valid_lines = 0
    valid_lines = []
    acum_length = 0
    for i in range(0,len(lines)):
        x1 = lines[i][0][0]
        y1 = lines[i][0][1]
        x2 = lines[i][0][2]
        y2 = lines[i][0][3]
        line_length = length(x1,x2,y1,y2)
        current_line = []
        slope = slope_cal(x1,x2,y1,y2)
        dist_to_ref = 0
        
        # Check if the line is vertical and with the desired length
        if slope == 1 and line_length > 60 and line_length < 75:
            # discard the lines that dont start where the blade should start
            if ( (centerY - y1) > 30 and (centerY - y1) < 46 ) or ((centerY - y2) > 30 and (centerY - y2) < 46):
                #valid_lines +=1
                acum_length += line_length
                valid_lines.append(lines[i][0])
                cv.line(img, (x1, y1), (x2, y2), (b, g, r), thickness, cv.LINE_AA)
                # Text print
                ytext = 0
                if y1<=y2:
                    ytext = y2
                    dist_to_ref = length(centerX,x2,centerY,y2)
                else:
                    ytext = y1
                    dist_to_ref = length(centerX,x1,centerY,y1)
                x_mean = (x1 + x2)/2
                lines_hist[int(x_mean)] += 1
                dist_to_ref = dist_to_ref*px_to_mm
                #cv.putText(img,str( "(" + str(x1) + ","  + str(ytext) + ")"),(x1 - 40,ytext + 50), font, 0.5,(0,0,255),1,cv.LINE_AA)
                #cv.putText(img,str(line_length),(x1 - 20,ytext + 70), font, 0.5,(0,0,255),1,cv.LINE_AA)
                cv.putText(img,str(str('%.2f'% dist_to_ref) + "mm"),(x1 - 20,ytext + 90), font, 0.5,(0,0,255),1,cv.LINE_AA)
                #cv.putText(img,str( "(" + str(x1) + ","  + str(ytext) + ")"),(x1 - 40,ytext + 50), font, 0.5,(0,0,255),1,cv.LINE_AA)
                cv.drawMarker(img,(x1,int(y1 -   line_length/3)),(0,255,0),cv.MARKER_TILTED_CROSS ,10,2,cv.LINE_AA)
                cv.drawMarker(img,(x1,int(y1 - 2*line_length/3)),(0,255,0),cv.MARKER_TILTED_CROSS ,10,2,cv.LINE_AA)                
    #cv.putText(img,str("Lines: " + str(valid_lines)),(40,40), font, 1,(0,0,0),2,cv.LINE_AA)
    #plt.hist(lines_hist)
    #plt.draw()
    avg_length = acum_length/len(valid_lines)
    cv.putText(img,str("avg_ length: " + str('%.2f' %avg_length)),(40,70), font, 1,(0,0,0),2,cv.LINE_AA)        
    cv.putText(img,str("Lines: " + str(len(valid_lines))),(40,40), font, 1,(0,0,0),2,cv.LINE_AA)        

    return img, lines_hist


def slope_cal(x0,x1,y0,y1):
    slope = np.arctan2((y1-y0),(x1-x0))
    if  np.abs(slope - 0) < slope_th or np.abs(slope - np.pi) < slope_th:
        return 0
    if np.abs(slope - np.pi/2) < slope_th or np.abs(slope + np.pi/2) < slope_th:
        return 1
    return 2


def length(x0,x1,y0,y1):
    return np.sqrt(np.power(y1-y0,2) + np.power(x1-x0,2))

def discard_lines(valid_lines, img_width):


    print("holis")

if __name__ == '__main__':
    main(sys.argv)