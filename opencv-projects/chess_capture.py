from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv

camera = PiCamera()


time.sleep(0.1)

for i in range(20):
	print("Be ready for the next picture")
	time.sleep(2)
	print("Taking picture...")
	time.sleep(0.3)
	rawCapture = PiRGBArray(camera)
	camera.capture(rawCapture, format='bgr')
	image = rawCapture.array
	name = 'imgs/chessboard/chess_test_' + str(i) + '.jpg'
	cv.imwrite(name,image)
	print("File saved with name: " + name)

