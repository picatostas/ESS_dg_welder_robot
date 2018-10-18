import cv2
import numpy as np
import sys
#variables
#ddepth = cv2.CV_16S
#kernel_size = 5
minLineLength = 50
maxLineGap = 300
lowTh = 100
HighTh = 300
#source img
#img = cv2.imread('imgs/grid.jpg')
#img = cv2.imread('grid_rpi_black.jpg')
if len(sys.argv) < 2:
	print(" the img is missing as an argument")
	exit(0)

img =cv2.imread(sys.argv[1])


grid = img
# gray scale
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#gaussian blur
blur = cv2.GaussianBlur(gray, (3, 3), 1)
# canny detection
edges   = cv2.Canny(gray, lowTh, HighTh,apertureSize = 3)
canny = edges
edges1 = edges
#laplacian
#laplace = cv2.Laplacian(blur, ddepth, kernel_size) 

# get lines from canny source
lines = cv2.HoughLinesP(    edges,1,np.pi/180,50,None,minLineLength,maxLineGap)

for i in range(0,len(lines)):
	x1 = lines[i][0][0]
	y1 = lines[i][0][1]
	x2 = lines[i][0][2]
	y2 = lines[i][0][3]
	cv2.line(grid, (x1, y1), (x2, y2), (0, 255, 0), 3, cv2.LINE_AA)


# save imgs
#cv2.imwrite('gaussian.jpg',blur)
canny_out = 'canny' + sys.argv[1]
hough_out = 'hough' + sys.argv[1]
cv2.imwrite(canny_out,canny)
cv2.imwrite(hough_out,grid)
#cv2.imwrite('laplacian.jpg',laplace)
