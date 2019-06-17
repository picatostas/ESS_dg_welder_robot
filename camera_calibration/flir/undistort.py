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

images = glob.glob('*.jpg')

for fname in images:
    # read image
    img = cv2.imread(fname)
    
    h = np.size(img, 0)
    w = np.size(img, 1)

    # undistort
    newcamera, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0)

    mapx, mapy = cv2.initUndistortRectifyMap(K,d,None,newcamera,(w,h),5)
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
    
    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('calibrated_' + fname,dst)
