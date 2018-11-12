import numpy as np
import cv2
import glob
import yaml

# copy parameters to arrays
#K = np.array([[1588.8406075452856, 0.0, 968.4316269781474],[0.0, 1472.3533526712627, 446.3351454689702],[0.0, 0.0, 1.0]])
#d = np.array([1,1,1,1,1])
# just use first two terms

with open('calibration.yaml') as f:
    calibration = yaml.load(f)
matrix  = calibration['camera_matrix']
distort = calibration['dist_coeff']
K = np.array(matrix)
d = np.array(distort)

images = glob.glob('chess_test_10.jpg')

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