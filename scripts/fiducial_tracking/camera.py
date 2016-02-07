#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib as plt
import math
import rospy
import time

from std_msgs.msg import UInt8
from geometry_msgs.msg import Point

from calibrate import *

class Vision():
    def __init__(self):
        # Intializes video capture variables
        self.cap = cv2.VideoCapture(0)
        self.frame_width = self.cap.get(3)
        self.frame_height = self.cap.get(4)
        
        # Initializes ROS variables
        rospy.init_node('vision_node')
        self.sub_mode = rospy.Subscriber('mode', UInt8, self.mode_callback)
        self.pub_fiducial = rospy.Publisher('/vision', Point, queue_size=10)

        # Main loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mode == 3:
                self.track_object() # Camshift
            else:
                self.find_squares() # Square detection
#            else:
#                ret, img = self.cap.read()
#                cv2.imshow('camera', img)

            # Displays images for camshift (mode = 3) and square detection (mode = 5)
            if self.img != None:
                cv2.imshow('camera', self.img)
            if self.canny != None:
                cv2.imshow('canny', self.canny)

            cv2.waitKey(1)
            r.sleep()

    # Acquires drone's current mode
    def mode_callback(self, data):
        if self.mode != data.data:
            cv2.destroyAllWindows()

        self.mode = data.data

if __name__ == '__main__':
    try:
        var = Vision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
