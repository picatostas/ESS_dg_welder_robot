import cv2
import numpy as np
import glob
import yaml


with open('calibration.yaml') as f:
    calibration = yaml.load(f)
matrix = calibration['camera_matrix']
distort = calibration['dist_coeff']
mtx = np.array(matrix)
dist = np.array(distort)
# squeare size in mm
square_size = 25
# chessboard size
chess_rows = 6
chess_columns = 9
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS +
            cv2.TERM_CRITERIA_MAX_ITER, square_size, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chess_rows*chess_columns, 3), np.float32)
objp[:, :2] = np.mgrid[0:chess_columns, 0:chess_rows].T.reshape(-1, 2)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
axis_cube = np.float32([[0, 0, 0], [0, 3, 0], [3, 3, 0], [3, 0, 0],
                        [0, 0, -3], [0, 3, -3], [3, 3, -3], [3, 0, -3]])


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


def draw_cube(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)
    # draw pillars in blue color
    for i, j in zip(range(4), range(4, 8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)
    return img


print("Object points prepared")
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('*.jpg')
print(str(len(images)) + " images found")


for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(
        gray, (chess_columns, chess_rows), None)

    if ret == True:
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)

        # Find the rotation and translation vectors.
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(
            objp, corners2, mtx, dist)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        imgpts_cube, jac = cv2.projectPoints(
            axis_cube, rvecs, tvecs, mtx, dist)
        img_posed = draw(img, corners2, imgpts)
        cv2.imwrite('posed_' + fname, img_posed)

        ## Render cube
        #img_1 = img
        #img_cube = draw_cube(img_1,corners2,imgpts_cube)
        #cv2.imwrite('cube ' + fname, img_cube)
