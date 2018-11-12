# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import yaml 
import glob
import numpy as np

#Houglines detection
minLineLength = 30
maxLineGap = 50
#canny detection variables
lowTh = 100
HighTh = 300
slope_th = 0.044
# extract distortion parameters from yaml file
with open('./imgs/chessboard/calibration1.yaml') as f:
    calibration = yaml.load(f)
matrix  = calibration['camera_matrix']
distort = calibration['dist_coeff']
K = np.array(matrix)
d = np.array(distort)
#HSV green mask parameters
sensitivity = 20
lower_green = np.array([60 - sensitivity,10,10])
upper_green = np.array([60 + sensitivity,170,170])
# initialize the camera and grab a reference to the raw camera capture
res_v = 480
res_h = 320

camera = PiCamera()
camera.resolution = (res_v, res_h)
camera.framerate = 10
camera.brightness = 60
rawCapture = PiRGBArray(camera, size=(res_v, res_h))

# allow the camera to warmup
time.sleep(0.1)
def main():
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# grab the raw NumPy array representing the image, then initialize the timestamp
		# and occupied/unoccupied text
		image = frame.array
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
				#if slope is 2:
				#	cv.line(image, (x1, y1), (x2, y2), (255, 0, 0), 1, cv.LINE_AA)
		#print(str(len(h_lines)) + "\t" + str(len(v_lines)))
		# show the frame
		cv.imshow("Frame", image)
		#cv.imshow("Canny", edges)
		#cv.imshow("Green Mask",green_mask)
		key = cv.waitKey(1) & 0xFF

		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)

		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break
def slope_cal(x0,x1,y0,y1):
	slope = np.arctan2((y1-y0),(x1-x0))
	if  np.abs(slope - 0) < slope_th or np.abs(slope - np.pi) < slope_th:
		return 0
	if np.abs(slope - np.pi/2) < slope_th or np.abs(slope + np.pi/2) < slope_th:
		return 1
	return 2
def length(x0,x1,y0,y1):
	return np.sqrt(np.power(y1-y0,2) + np.power(x1-x0,2))
main()
