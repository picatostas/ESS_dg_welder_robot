import numpy as np
import cv2
import glob
# squeare size in mm 
square_size = 25 
# chessboard size 
chess_rows = 6
chess_columns = 8
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, square_size, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chess_rows*chess_columns,3), np.float32)
objp[:,:2] = np.mgrid[0:chess_columns,0:chess_rows].T.reshape(-1,2)
print("Object points prepared")
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.jpg')
print(str(len(images)) +" images found")
for i in range(len(images)):
	print(images[i])
for fname in images:
	print("Image " + fname + " is being processed")
	img = cv2.imread(fname)
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	# Find the chess board corners
	ret, corners = cv2.findChessboardCorners(gray, (chess_columns,chess_rows),None)
	# If found, add object points, image points (after refining them)
	if ret == True:
		objpoints.append(objp)
		corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
		imgpoints.append(corners)
		# Draw and display the corners
		#img = cv2.drawChessboardCorners(img, (chess_columns, chess_rows), corners,ret)
		img = cv2.drawChessboardCorners(img, (chess_columns, chess_rows), corners2,ret)
		#cv2.imshow(img)
		cv2.imwrite('processed' + fname,img)
