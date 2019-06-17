
import numpy as np
import cv2
import glob
import yaml

with open('calibration.yaml') as f:
    calibration = yaml.load(f)
matrix  = calibration['camera_matrix']
distort = calibration['dist_coeff']
K = np.array(matrix)
d = np.array(distort)

images = glob.glob('*.png')

for fname in images:
    # read image
    img = cv2.imread(fname)
    h = np.size(img, 0)
    w = np.size(img, 1)
    # undistort
    newcamera, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0)
    newimg = cv2.undistort(img, K, d, None, newcamera)

    # save image
    newfname = 'undistorted_' + fname
    cv2.imwrite(newfname, newimg)
