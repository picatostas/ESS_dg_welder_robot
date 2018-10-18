from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv

camera = PiCamera()
rawCapture = PiRGBArray(camera)

time.sleep(0.1)

camera.capture(rawCapture, format='bgr')
time.sleep(0.1)
image = rawCapture.array

cv.imwrite('camera_test.jpg',image)
