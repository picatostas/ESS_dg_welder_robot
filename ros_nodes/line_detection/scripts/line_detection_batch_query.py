#!/usr/bin/env python

# Python libs
import sys, time, glob, threading
# numpy and scipy
import numpy as np
from scipy.ndimage import filters
# OpenCV
import cv2 as cv
# Ros libraries
import roslib, rospy, imutils
import matplotlib.pyplot as plt
# Ros Messages
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
#Houglines detection threshold values in px
minLineLength = 10
maxLineGap = 30
#canny detection variables
len_th = [60,75]
dst_th = [30,60]
canny_th = [50,300]# 300 for real grid, 200 for template
slope_th = 0.1 #threshold value for considering vertical lines, in rads 0.044
px_to_mm = 0.1499 # pseudo empiric value

# Macros for lines/blade sort
LINES_PER_BLADE = 5
FRAMES_NUMBER = 5
BLADES_PER_GRID = 16 #CSPEC
loc_th = 20

chunk_factor = 3 # as the camera only frames 3 grids at a time 
image_chunks = []
img_path = '/home/multigrid/catkin_ws/src/line_detection/scripts/'
font = cv.FONT_HERSHEY_SIMPLEX

class image_feature:

    def __init__(self):
        self.ready = True
        #### values for 1280x960 default from raspicam_node/camera_info
        self.camera_matrix = np.array([[1014.343103379204, 0, 637.2463708126373],[0, 1011.69373183754, 469.5663779911617],[0.0, 0.0, 1.0]])
        self.dist_coeff    = np.array([0.1570058008946036, -0.2862704919204555, -5.60164774255961e-05, 0.001586362091473342, 0])
        self.frames = []
        self.store_frames = False
        self.processed_frames = 0
        self.grid_ref = np.zeros((3,2))
        self.marker_count = np.zeros(3)
        self.blades_ready = True
        self.preproc_time = 0
        self.blade_lines        = [[[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []],
                                   [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []],
                                   [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []]]
        self.blade_lines_sorted = self.blade_lines
        self.blade_count        = np.zeros((3,16))
        ## This values are in pixes for 1080x960 res and a camera position of z30 and aprox 154 mm from grid 
        self.blade_loc = 89.58 , 153.81 , 221.76 , 285.96 , 354.84 , 421.57 , 489.30 , 555.31 , 621.92 , 687.86 , 753.63 , 819.42 , 883.50 , 948.99 , 1016.79 , 1081.44

        self.detect_query = rospy.Subscriber("/detection_query",String,
             self.query_callback , queue_size = 10)

        self.image_pub = rospy.Publisher("/lines_detected/compressed",
            CompressedImage, queue_size = 1)
        self.blades_pub = rospy.Publisher("/blades_location", String, queue_size = 40)
        self.subscriber = None # otherwise it doesnt work, as I need to resub each time that i got a query
        ### Load images and get some features
        self.template = cv.imread(img_path + 'pattern.png')
        self.template = cv.cvtColor(self.template, cv.COLOR_BGR2GRAY)
        self.template = cv.Canny(self.template, canny_th[0], canny_th[1])
        self.han_logo = cv.imread(img_path + 'han100.png')
        self.ess_logo = cv.imread(img_path + 'ess100.png')
        self.han_logo_h = np.size(self.han_logo,0)
        self.han_logo_w = np.size(self.han_logo,1)
        self.ess_logo_h = np.size(self.ess_logo,0)
        self.ess_logo_w = np.size(self.ess_logo,1)
        self.thread_list = []


    def query_callback(self, ros_data):

        message = ros_data.data
        if message == 'q':
            print("Detection queried")
            ### Empty frame buffer reset the count
            self.blade_lines        = [[[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []],
                                       [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []],
                                       [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []]]
            self.frames[:]   = []            
            self.grid_ref    = np.zeros((3,2))
            self.blade_count = np.zeros((3,16))
            self.processed_frames = 0
            self.store_frames = True
            self.preproc_time = time.time()
            self.suscribe()


    def suscribe(self):
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.frame_callback,  queue_size = 10)
    

    def unsub(self):
        self.subscriber.unregister()
    

    def frame_callback(self, ros_data):
        if self.store_frames:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
            self.frames.append(image)
            if len(self.frames) == FRAMES_NUMBER:
                self.process_frames()

    ## for all the possible lines found for a blade in a grid, average
    def sort_lines(self):
        blades_msg = [[[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []],
                      [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []],
                      [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []]]
        for grid_idx, grid in enumerate(self.blade_lines):
            for blade_idx, blade in enumerate(grid):
                line_avg = np.zeros(4)            
                for line in blade:
                    # Check which of the points that make the line is lower so we store them properly
                    if line[3] > line[1]:
                        line_avg[0] += line[2]
                        line_avg[1] += line[3]
                        line_avg[2] += line[0]
                        line_avg[3] += line[1]
                    else:
                        line_avg[0] += line[0]
                        line_avg[1] += line[1]
                        line_avg[2] += line[2]
                        line_avg[3] += line[3]
                # mean of all the points, least mean square could be used instead
                line_avg /= len(blade)
                self.blade_lines_sorted[grid_idx][blade_idx] = line_avg                
                blades_msg[grid_idx][blade_idx].append('%.2f' % (( self.grid_ref[grid_idx][0] - line_avg[0] )*px_to_mm ))
                blades_msg[grid_idx][blade_idx].append('%.2f' % (( self.grid_ref[grid_idx][1] - line_avg[1] )*px_to_mm ))
                blades_msg[grid_idx][blade_idx].append('%.2f' % (( self.grid_ref[grid_idx][0] - line_avg[2] )*px_to_mm ))
                blades_msg[grid_idx][blade_idx].append('%.2f' % (( self.grid_ref[grid_idx][1] - line_avg[3] )*px_to_mm ))

        for grid_idx, grid in enumerate(blades_msg):
            #self.blades_pub.publish(str("Grid " + str(grid_idx + 1)))
            for blade_idx, blade in enumerate(grid):
                message = "G:" + str(grid_idx) + " B:" + str(blade_idx) +" "+ str(blade)
                self.blades_pub.publish(message)
            rospy.sleep(0.1)
                #print()
        #print("Sorting finished")

    def process_chunk(self, chunk, chunk_idx, chunk_size, chunk_time):
        #print("Thread " + str(chunk_idx) + " started")
        edges = cv.Canny(chunk, canny_th[0], canny_th[1],apertureSize = 3)
        lines = cv.HoughLinesP(edges,1,np.pi/180,40,None,minLineLength,maxLineGap)
        if lines is not None:
            ctr   = match_template(self.template, chunk)
            #print ctr
            self.grid_ref[chunk_idx][0] += ctr[0]
            self.grid_ref[chunk_idx][1] += ctr[1] #+ chunk_size*chunk_idx
            # To keep track of how many times the chunk has been processed
            self.marker_count[chunk_idx] += 1
            get_lines(self,ctr,lines,chunk_idx,chunk_size)
        #print("Elapsed time for chunk %d, %.2f millis" %(chunk_idx, ((time.time() - chunk_time)*1000) ))


    def process_frames(self):
        print("Frames received : " + str(len(self.frames)))
        ### All the frames are equal and share the same distortion parameters
        h = np.size(self.frames[0],0)
        w = np.size(self.frames[0],1)
        K = self.camera_matrix
        d = self.dist_coeff
        newcamera, roi = cv.getOptimalNewCameraMatrix(K,d,(w,h),0)
        ###
       
        chunk_size = h / chunk_factor
        for image in self.frames:
            frame_time = time.time()
            ### Camera distortion correction
            image = cv.undistort(image, K,d, None, newcamera)
            gray  = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
            self.thread_list = []
            image_chunks = np.vsplit(gray,chunk_factor)
            for chunk_idx, chunk in enumerate(image_chunks):
                # Stop processing that chunk if grid has been already detected 
                grid_ready = True
                for blade in self.blade_count[chunk_idx]:
                    if blade != LINES_PER_BLADE: grid_ready = False
                if grid_ready: continue
                # Time stamp for each chunck
                chunk_time = time.time()    
                thread = threading.Thread(target = self.process_chunk, args = (chunk, chunk_idx, chunk_size,chunk_time))
                thread.start()
                self.thread_list.append(thread)
            # Wait for all threads to finish before moving to next frame
            for thread in self.thread_list:
                thread.join()
            self.processed_frames += 1

            print("Elapsed time for frame %d, %.2f millis" %(self.processed_frames, ((time.time() - frame_time)*1000) ))
            #print("Frames processed: " + str(self.processed_frames))
            # Stop processing frames if detection ready
            detection_ready = True
            for idx, grid in enumerate(self.blade_count):
                for blade in grid:
                    if blade != LINES_PER_BLADE:
                        detection_ready = False                
            if detection_ready: break

        #print("Detection finished")
        print("Elapsed time for all frames %.2f millis" %((time.time() - self.preproc_time)*1000))
        ### save last frame
        image = self.frames[-1]
        ### empty frame buffer after processing
        self.frames[:] = []

        enough_lines = True

        for grid in self.blade_lines:
            for blade in grid:  
                if (len(blade) < LINES_PER_BLADE):
                    enough_lines = False
                    break

        if enough_lines is False:
            ## get more frames
            print("Not enough detected lines,Getting more frames...")       
            return
        else:
            print("Elapsed time for detecting all grids %.2f millis" %((time.time() - self.preproc_time)*1000))
            self.unsub()
            self.blade_count = np.zeros((3,16))
            print("Enough lines per blade detected, calculating welding points")
            for i, marker in enumerate(self.grid_ref):
                marker /= self.marker_count[i]
            self.marker_count = np.zeros(3)
            #print(self.grid_ref)
            self.sort_lines()

            ## Draw lines and markers location 
            for chunk_idx, grid in enumerate(self.blade_lines_sorted):
                for blade in grid:
                        cv.line(image, (int(blade[0]),int(blade[1]) +chunk_size*chunk_idx ), (int(blade[2]),int(blade[3]) + chunk_size*chunk_idx), (200, 0, 200), 2, cv.LINE_AA)

            for idx, ctr in enumerate(self.grid_ref):            
                cv.drawMarker(image,(int(ctr[0]),int(ctr[1])+ chunk_size*idx),(0,255,0),cv.MARKER_CROSS ,60,2,cv.LINE_AA)
                cv.circle(    image,(int(ctr[0]),int(ctr[1])+ chunk_size*idx), 30, (0,255,0),2,cv.LINE_AA)

            ### IMAGE HEADER    
            if self.han_logo_h > self.ess_logo_h:
                image[0:self.han_logo_h,0:w] = [255,255,255]
            else:
                image[0:self.ess_logo_h,0:w] = [255,255,255]
    
            image[0:self.han_logo_h,int(0.72*w):int(0.72*w + self.han_logo_w)] = self.han_logo
            image[0:self.ess_logo_h,int(0.15*w):int(0.15*w + self.ess_logo_w)] = self.ess_logo
            cv.putText(image,str("MG Welder Robot : Han Solo"),(420,40), font, 1,(0,0,0),3,cv.LINE_AA)
            ### 
    
            ### Create and Publish CompressedIamge ####
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv.imencode('.jpg', image)[1]).tostring()
            self.image_pub.publish(msg)
            ### 
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
        edged = cv.Canny(resized,canny_th[0], canny_th[1])
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
    return [(startX + endX)/2 , (startY + endY)/2]


def get_lines(self,ctr, lines, chunk_idx,chunk_size):
    for element in lines:
        line = element[0]
        x1   =    line[0]
        y1   =    line[1]
        x2   =    line[2]
        y2   =    line[3]

        line_len =    length(x1,x2,y1,y2)
        slope    = slope_cal(x1,x2,y1,y2)
        
        # Check if the line is vertical and with the desired length
        if slope == 1 and line_len > len_th[0] and line_len < len_th[1]:
            # discard the lines that dont start where the blade should start
            if ( (ctr[1] - y1) > dst_th[0] and (ctr[1] - y1) < dst_th[1] ) or ((ctr[1] - y2) > dst_th[0] and (ctr[1] - y2) < dst_th[1]):
                
                if y1<=y2:
                    dist_to_ref = length(ctr[0],x2,ctr[1],y2)
                else:
                    dist_to_ref = length(ctr[0],x1,ctr[1],y1)

                for loc_idx, loc in enumerate(self.blade_loc):
                    if (len(self.blade_lines[chunk_idx][loc_idx]) < LINES_PER_BLADE):
                        if (dist_to_ref > (loc - loc_th) ) and (dist_to_ref < (loc + loc_th)):
                            #cv.line(img, (line[0],line[1] + chunk_idx*chunk_size), (line[2],line[3] + chunk_idx*chunk_size), (0, 0, 255), 3, cv.LINE_AA)
                            self.blade_count[chunk_idx][loc_idx] += 1
                            self.blade_lines[chunk_idx][loc_idx].append(line)
    #return img     


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