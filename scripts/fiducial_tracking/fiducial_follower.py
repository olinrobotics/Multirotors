#!/usr/bin/env python
import rospy
import roslib
import time
import cv2
import numpy as np
roslib.load_manifest('mavros')

from sensor_msgs.msg import Image, Joy, CameraInfo
from geometry_msgs.msg import Point
from mavros_msgs.msg import BatteryStatus, State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge, CvBridgeError

from drone import *

class FiducialFollower():
    def __init__(self, drone):
        rospy.init_node('fiducial_follower')
        self.drone = drone

        # Camera/vision variables
        self.cap = cv2.VideoCapture(0)
        self.fiducial = (0, 0, 0)
        self.bridge = CvBridge()

        # ROS publishers
        self.pub_image = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.pub_caminfo = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)

    """ Publishes image to image_raw """
    def run(self):
        ret, cv_image = self.cap.read()
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.pub_image.publish(ros_image)

        self.fly()

    """ WIP - control algorithms """
    def fly(self):
        pass

    """ Checks if drone has landed """
    def finished(self):
        return False


if __name__ == '__main__':
    try:
        var = FiducialFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
