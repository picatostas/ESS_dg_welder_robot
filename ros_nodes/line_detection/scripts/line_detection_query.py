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
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
#Houglines detection threshold values in px
minLineLength = 10
maxLineGap = 30
#canny detection variables
lowTh = 50
HighTh = 300 # 300 for real grid, 200 for template
slope_th = 0.044 #threshold value for considering vertical lines, in rads
px_to_mm = 0.1499 # pseudo empiric value

# Macros for lines/blade sort
LINES_PER_BLADE = 20
FRAMES_NUMBER = 30

font = cv.FONT_HERSHEY_SIMPLEX

class image_feature:

    def __init__(self):
        self.ready = True
        #### values for 1280x960 default from raspicam_node/camera_info
        self.camera_matrix = np.array([[1014.343103379204, 0, 637.2463708126373],[0, 1011.69373183754, 469.5663779911617],[0.0, 0.0, 1.0]])
        self.dist_coeff    = np.array([0.1570058008946036, -0.2862704919204555, -5.60164774255961e-05, 0.001586362091473342, 0])
        self.frames = []
        self.grid_ref = [0,0]
        self.blades_ready = True
        self.lines_hist = range(0,1280)
        self.lines_hist[:] = [0] * len(self.lines_hist)
        self.blade_dist = []
        self.blade_hist = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
        self.template = cv.imread('/home/multigrid/catkin_ws/src/line_detection/scripts/pattern.png')
        ## This values are in pixes for 1080x960 res and a camera position of z30 and aprox 154 mm from grid 
        self.blade_loc = 89.58 , 153.81 , 221.76 , 285.96 , 354.84 , 421.57 , 489.30 , 555.31 , 621.92 , 687.86 , 753.63 , 819.42 , 883.50 , 948.99 , 1016.79 , 1081.44
        self.detect_query = rospy.Subscriber("/detection_query",String,
             self.query_callback , queue_size = 10)

        self.image_pub = rospy.Publisher("/lines_detected/compressed",
            CompressedImage, queue_size = 1)
        self.blades_pub = rospy.Publisher("/blades_location", String, queue_size = 10)
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.frame_callback,  queue_size = 10)
        self.subscriber.unregister()

    def query_callback(self, ros_data):

        message = ros_data.data
        if message == 'q':
            print("Detection queried")
            self.frames[:] = []
            self.suscribe()


    def suscribe(self):
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.frame_callback,  queue_size = 10)
    

    def unsub(self):
        self.subscriber.unregister()
    

    def frame_callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        self.frames.append(image)
        if len(self.frames) == FRAMES_NUMBER:
            self.blade_hist = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
            self.detect_lines()
            self.unsub()

    ## for all the possible lines found for a blade, average
    def sort_lines(self, img):
        lines_ref = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
        for blades in range(len(self.blade_hist)):
            x1_acum = 0
            y1_acum = 0
            x2_acum = 0
            y2_acum = 0
            line = []            
            for lines in range(len(self.blade_hist[blades])):
                # Check which of the points that make the line is lower so we store them properly
                if self.blade_hist[blades][lines][3] > self.blade_hist[blades][lines][1]:
                    x1_acum += self.blade_hist[blades][lines][2]
                    y1_acum += self.blade_hist[blades][lines][3]
                    x2_acum += self.blade_hist[blades][lines][0]
                    y2_acum += self.blade_hist[blades][lines][1]
                else:
                    x1_acum += self.blade_hist[blades][lines][0]
                    y1_acum += self.blade_hist[blades][lines][1]
                    x2_acum += self.blade_hist[blades][lines][2]
                    y2_acum += self.blade_hist[blades][lines][3]
            # mean of all the points, least mean square could be used instead
            if len(self.blade_hist[blades]) is not 0:
                x1_acum /= len(self.blade_hist[blades])
                y1_acum /= len(self.blade_hist[blades])
                x2_acum /= len(self.blade_hist[blades])
                y2_acum /= len(self.blade_hist[blades])
                line.append('%.2f' % (( x1_acum - self.grid_ref[0] )*px_to_mm*(-1) ))
                line.append('%.2f' % (( y1_acum - self.grid_ref[1] )*px_to_mm*(-1) ))
                line.append('%.2f' % (( x2_acum - self.grid_ref[0] )*px_to_mm*(-1) ))
                line.append('%.2f' % (( y2_acum - self.grid_ref[1] )*px_to_mm*(-1) ))
                lines_ref[blades].append(line)
            ## Draw detected lines lines in image
            cv.line(img, (x1_acum, y1_acum), (x2_acum, y2_acum), (200, 0, 200), 1, cv.LINE_AA)

            ## Draw position of tig welder spots
            line_length = length(x1_acum,x2_acum,y1_acum,y2_acum)
            cv.drawMarker(img,(x1_acum,int(y1_acum -   line_length/3)),(0,255,0),cv.MARKER_TILTED_CROSS ,10,2,cv.LINE_AA)
            cv.drawMarker(img,(x1_acum,int(y1_acum - 2*line_length/3)),(0,255,0),cv.MARKER_TILTED_CROSS ,10,2,cv.LINE_AA)
        print("Line points referenced from Marker")
        blades_msg = np.array(lines_ref)
        for blade in range(len(blades_msg)):
            print("Blade n: " +str(blade)+" "+ str(blades_msg[blade]))

        publish_blades = True

        for blade in blades_msg:

            if blade[0] is None:

                publish_blades = False
                print("Publish cancelled, a blade wasnt found")
                break

        if publish_blades:
            for blade in range(len(blades_msg)):   
                self.blades_pub.publish(str(blades_msg[blade][0]))

        return img


    def detect_lines(self):
        print(" Frames received : " + str(len(self.frames)))
        center_avg = [0,0]
        for image in self.frames:
            ### Camera distortion correction
            h = np.size(image,0)
            w = np.size(image,1)
            # empty the line hist buffer
            lines_hist = [0]*w
            K = self.camera_matrix
            d = self.dist_coeff
            newcamera, roi = cv.getOptimalNewCameraMatrix(K,d,(w,h),0)
            image = cv.undistort(image, K,d, None, newcamera)
            ###
            ### canny detection
            gray  = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
            edges = cv.Canny(gray, lowTh, HighTh,apertureSize = 3)
            lines = cv.HoughLinesP(edges,1,np.pi/180,40,None,minLineLength,maxLineGap)
            ### Pattern recognition 
            startX, startY, endX, endY = match_template(self.template, image)
            centerX = (startX + endX)/2
            centerY = (startY + endY)/2
            center_avg[0] += centerX
            center_avg[1] += centerY
            ###
            if lines is not None:
                image = calculate_lines(self,centerX,centerY,lines,image,0,0,255,2)
            cv.putText(image,str("MG Welder Robot"),(550,40), font, 1,(0,0,0),3,cv.LINE_AA)
            
            ### Marker for the detected reference pattern
            #cv.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 1)
            cv.drawMarker(image,(centerX,centerY),(0,255,0),cv.MARKER_CROSS ,40,1,cv.LINE_AA)
            cv.circle(image,(centerX,centerY), 20, (0,255,0),1,cv.LINE_AA) 
            ###
        self.grid_ref[0]  =  center_avg[0]/len(self.frames)
        self.grid_ref[1]  =  center_avg[1]/len(self.frames)

        self.blade_dist = sorted(self.blade_dist)
        for lines in range((len(self.blade_hist))):
            print("lines found in blade n " + str(lines) + " : " + str(len(self.blade_hist[lines])) )
        self.ready = True
        for i in range(len(self.blade_hist)):
            # Cannot resuscribe from here to the topic, dont know why
            # in order to get more frames if with 10 it wasnt enough for detecting all the blades
            if (len(self.blade_hist[i]) < LINES_PER_BLADE):
                print("Not enough detected lines, more frames may be needed...")
                #self.blades_ready = False
                self.ready = False
                break
        if self.ready:
            print("Detection finished")
        image = self.sort_lines(image)        
        ### Create and Publish CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv.imencode('.jpg', image)[1]).tostring()
        # Publish new image into the topic
        self.image_pub.publish(msg)
        ###
        #cv.imshow('canny', canny )
        #cv.destroyAllWindows()
        # Just to check the histogram of lines detected by blade
        #plt.plot(self.lines_hist)
        #plt.show()    

def main(args):
    ic = image_feature()
    #if not ic.blades_ready:
    #    ic.suscribe()
    #    ic.blades_ready = True
    rospy.init_node('line_detection', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv.destroyAllWindows()


def match_template(template, image):
    template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    template = cv.Canny(template, lowTh, HighTh)
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
        edged = cv.Canny(resized,lowTh, HighTh)
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


def calculate_lines(self,centerX,centerY,lines, img,r = 255,g = 0,b = 0, thickness = 1):
    #valid_lines = []
    acum_length = 0
    for i in range(0,len(lines)):
        line = []
        x1 = lines[i][0][0]
        y1 = lines[i][0][1]
        x2 = lines[i][0][2]
        y2 = lines[i][0][3]
        
        line_length = length(x1,x2,y1,y2)
        slope = slope_cal(x1,x2,y1,y2)
        dist_to_ref = 0
        
        # Check if the line is vertical and with the desired length
        if slope == 1 and line_length > 60 and line_length < 75:
            # discard the lines that dont start where the blade should start
            if ( (centerY - y1) > 30 and (centerY - y1) < 46 ) or ((centerY - y2) > 30 and (centerY - y2) < 46):

                line.append(x1)
                line.append(y1)
                line.append(x2)
                line.append(y2)
                #acum_length += line_length
                #valid_lines.append(lines[i][0])
                
                ytext = 0
                if y1<=y2:
                    ytext = y2
                    dist_to_ref = length(centerX,x2,centerY,y2)
                else:
                    ytext = y1
                    dist_to_ref = length(centerX,x1,centerY,y1)
                x_mean = (x1 + x2)/2
                # Line count for histogram
                self.lines_hist[int(x_mean)] += 1

                for j in range(len(self.blade_loc)):
                    if (len(self.blade_hist[j]) < LINES_PER_BLADE):
                        if (dist_to_ref > (self.blade_loc[j] - 20) ) and (dist_to_ref < (self.blade_loc[j] + 20)):
                                self.blade_hist[j].append(line)

                dist_trunc = float(('%.2f'% dist_to_ref))
                self.blade_dist.append(dist_trunc)
                dist_to_ref = dist_to_ref*px_to_mm                
                #cv.putText(img,str(str('%.2f'% dist_to_ref) + "mm"),(x1 - 20,ytext + 90), font, 0.5,(0,0,255),1,cv.LINE_AA)
                #cv.drawMarker(img,(x1,int(y1 -   line_length/3)),(0,255,0),cv.MARKER_TILTED_CROSS ,10,2,cv.LINE_AA)
                #cv.drawMarker(img,(x1,int(y1 - 2*line_length/3)),(0,255,0),cv.MARKER_TILTED_CROSS ,10,2,cv.LINE_AA)
                
    #cv.putText(img,str("Lines: " + str(valid_lines)),(40,40), font, 1,(0,0,0),2,cv.LINE_AA)
    #avg_length = acum_length/len(valid_lines)
    #cv.putText(img,str("avg_ length: " + str('%.2f' %avg_length)),(40,70), font, 1,(0,0,0),2,cv.LINE_AA)        
    #cv.putText(img,str("Lines: " + str(len(valid_lines))),(40,40), font, 1,(0,0,0),2,cv.LINE_AA)        
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