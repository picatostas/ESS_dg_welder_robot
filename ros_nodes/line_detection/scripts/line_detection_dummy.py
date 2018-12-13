#!/usr/bin/env python

# Python libs
import sys, time
import glob

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# Ros libraries
import roslib
import rospy
import imutils
import matplotlib.pyplot as plt
# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class image_feature:

    def __init__(self):
        self.ready = True
        #### values for 1280x960 default from raspicam_node/camera_info
        self.grid_ref = [0,0]
        self.blades_ready = True
        self.lines_hist = range(0,1280)
        self.lines_hist[:] = [0] * len(self.lines_hist)
        self.blade_dist = []
        self.dummy_blades = [['-12.29','6.30','-12.29','16.04'],
                             ['-22.19','6.45','-22.19','15.89'],
                             ['-32.38','6.45','-32.38','16.04'],
                             ['-42.42','6.45','-42.42','15.89'],
                             ['-52.76','6.45','-52.76','16.04'],
                             ['-62.81','6.45','-62.81','15.89'],
                             ['-73.00','6.30','-73.00','15.59'],
                             ['-82.89','6.30','-82.89','15.74'],
                             ['-93.09','6.45','-93.09','15.89'],
                             ['-102.83','6.45','-102.83','15.89'],
                             ['-112.72','6.45','-112.72','15.74'],
                             ['-122.62','6.45','-122.62','15.89'],
                             ['-132.36','6.45','-132.36','15.89'],
                             ['-142.41','6.30','-142.41','15.74'],
                             ['-152.00','6.30','-152.15','15.74'],
                             ['-161.89','6.45','-161.89','15.74']]

        self.detect_query = rospy.Subscriber("/detection_query",String,
             self.query_callback , queue_size = 10)
        self.blades_pub = rospy.Publisher("/blades_location", String, queue_size = 10)


    def query_callback(self, ros_data):

        message = ros_data.data
        if message == 'q':
            print("Detection queried")
            self.send_lines()

    def send_lines(self):
        blades_msg = np.array(self.dummy_blades)
        for blade in range(len(blades_msg)):
            self.blades_pub.publish(str(blades_msg[blade]))
 

def main(args):
    ic = image_feature()
    #if not ic.blades_ready:
    #    ic.suscribe()
    #    ic.blades_ready = True
    rospy.init_node('line_detection', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"



if __name__ == '__main__':
    main(sys.argv)