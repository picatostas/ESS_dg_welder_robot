import cv2
import numpy as np
#variables
ddepth = cv2.CV_16S
kernel_size = 5
minLineLength = 1
maxLineGap = 10

#source img
img = cv2.imread('imgs/grid.jpg')
# gray scale
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#gaussian blur
blur = cv2.GaussianBlur(gray, (3, 3), 1)
# canny detection
edges   = cv2.Canny(gray, threshold1 = 50, threshold2 = 200, apertureSize = 3)
canny = edges
edges1 = edges
#laplacian
laplace = cv2.Laplacian(blur, ddepth, kernel_size)

# get lines from canny source
lines = cv2.HoughLinesP(    edges,1,np.pi/180,200,minLineLength,maxLineGap)
for x1, y1, x2, y2 in lines[0]:
    cv2.line(edges, (x1, y1), (x2, y2), (255, 0, 0), 30)

lines = cv2.HoughLines(edges1,1,np.pi/180,200)
for rho,theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    cv2.line(edges1,(x1,y1),(x2,y2),(0,0,255),1)

# save imgs
#cv2.imwrite('gaussian.jpg',blur)
cv2.imwrite('canny.jpg',canny)
cv2.imwrite('canny_hough_p.jpg',edges)
cv2.imwrite('canny_hough.jpg',edges1)
cv2.imwrite('laplacian.jpg',laplace)