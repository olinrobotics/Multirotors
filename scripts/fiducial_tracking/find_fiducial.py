'''
    OLD CODE - needs to be replaced w/ ar_pose detection code

    This file should just publish the location of the fiducial
'''


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
        self.cap = cv2.VideoCapture(1)
        self.frame_width = self.cap.get(3)
        self.frame_height = self.cap.get(4)

        # Keeps track of images to display
        self.img = None
        self.canny = None

        # Keeps track of fiducial position and velocity
        self.x0 = 0.
        self.y0 = 0.

        self.x = 0.
        self.y = 0.
        self.z = 0.

        # Drone's current mode in finite state machine
        self.mode = 0

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

    # Tracks a selected object using the camshift algorithm
    def track_object(self):
        cv2.setMouseCallback('camera', self.draw_circle)
        ret, frame = self.cap.read()
        roi_corners = []
        roi_selected = False

        # Draws circles to indicate where the ROI is
        if len(roi_corners) < 2:
            for i in roi_corners:
                cv2.circle(frame, (i[0], i[1]), 5, (0, 0, 255), -1)

        # Sets up the camshift parameters, based on the selected ROI
        elif len(roi_corners) == 2:
            (x1, y1, x2, y2) = (roi_corners[0][0], roi_corners[0][1], roi_corners[1][0], roi_corners[1][1])
            track_window = (x1, y1, x2-x1, y2-y1)
            roi = frame[y1:y2, x1:x2]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_roi, np.array((0., 60., 30.)), np.array((180., 255., 255.)))
            roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0,180])
            cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
            term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
            
            # Resetting variables...
            roi_selected = True
            roi_corners = []

        # Tracks object, if a ROI has been selected
        if roi_selected:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            dst = cv2.calcBackProject([hsv], [0], roi_hist, [0,180], 1)

            ret, track_window = cv2.CamShift(dst, track_window, term_crit)

            # Draws box around tracked object
            pts = cv2.cv.BoxPoints(ret)
            pts = np.int0(pts)
            cv2.polylines(frame, [pts], True, (0, 255, 255))

        self.img = frame

    # Tracks a square fiducial
    def find_squares(self):
        # Captures and calibrates a frame
        ret, frame = self.cap.read()
        frame = calibrate(frame)
        img_display = np.copy(frame)
        filterCombination = 0


        while (filterCombination != 2):
            # Pre-processes image before tracking
            img = np.copy(frame)
            img = cv2.GaussianBlur(img, (3,3), 0)
            if filterCombination == 0:
                # lapl = cv2.Laplacian(img, cv2.CV_64F)
                # img = img - lapl
                img = cv2.inRange(img, np.array([125, 125, 125], dtype=np.uint8), np.array([255, 255, 255], dtype=np.uint8))
            img = cv2.Canny(img, 100, 150, apertureSize=5)
            retval, img = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
            self.canny = img

            # Locates connected components within the image
            contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Helper variables
            i1 = 0
            squares = []

            # Analyzes each contour: checks if it's a square, checks if it has at least 5 "children"
            for cnt in contours:
                # More helper variables
                children = []
                children_final = []
                children_areas = 0
                average_area = 0.0

                # Skips contours that are enormous or tiny
                if cv2.contourArea(cnt) > self.frame_height * self.frame_width * 0.4 or cv2.contourArea(cnt) < 100:
                    i1 += 1
                    continue

                # Appends all of the contour's children to an array (child = contour enclosed by a parent contour)
                if len(hierarchy[0]) > 0:
                    i2 = hierarchy[0][i1][2]
                    while i2 != -1:
                        children.append(contours[i2])
                        children_areas += cv2.contourArea(contours[i2])
                        i2 = hierarchy[0][i2][0]
                i1 += 1
       
                # The children must be similarly sized in the fiducial 
                if len(children) > 0:
                    average_area = float(children_areas) / len(children)
                    for cld in children:
                        if self.is_square(cld, 0.01) and abs(cv2.contourArea(cld) - average_area) < 100:
                            children_final.append(cld)

                # Checks if the contour is a square and if it contains at least 5 children
                cnt, cnt_square = self.is_square(cnt, 0.02) 
                if cnt_square and len(children_final) >= 5:
                    squares.append(cnt)

                    # Only tracks the smallest detected fiducial
                    if len(squares) == 2:
                        if filterCombination == 0:
                            if cv2.contourArea(squares[0]) > cv2.contourArea(squares[1]):
                                squares.pop(0)
                            else:
                                squares.pop(1)
                        elif filterCombination == 1:
                            if cv2.contourArea(squares[0]) < cv2.contourArea(squares[1]):
                                squares.pop(0)
                            else:
                                squares.pop(1)

            # Calculates the x, y coordinates and the area of the fiducial
            if len(squares) != 0:
                M = cv2.moments(np.array(squares))
                self.x0 = self.x
                self.y0 = self.y

                self.x = (int(M['m10'] / M['m00']) * 2.0 / self.frame_width) - 1.0
                self.y = (int(M['m01'] / M['m00']) * 2.0 / self.frame_height) - 1.0
                self.z = cv2.contourArea(squares[0])

                cv2.drawContours( img_display, squares, -1, (0, 255, 0), 2 )

                filterCombination = 2

            # Estimates the fiducial's position based on previous position data
            else:
                filterCombination += 1
                if filterCombination == 2:
                    dx = self.x - self.x0
                    dy = self.y - self.y0

                    self.x += dx
                    self.y += dy
                    self.x0 += dx
                    self.y0 += dy

                    if self.x > 0.2:
                        self.x = 0.2
                    elif self.x < -0.2:
                        self.x = 0.2
                    if self.y > 0.2:
                        self.y = 0.2
                    elif self.y < -0.2:
                        self.y = 0.2

                    circle_x = int( (self.x + 1) / 2 * self.frame_width )
                    circle_y = int( (self.y + 1) / 2 * self.frame_height )
                    cv2.circle( img_display, (circle_x, circle_y), 20, (255, 0, 0), 2 )                    

        fiducial_msg = Point()
        (fiducial_msg.x, fiducial_msg.y, fiducial_msg.z) = (self.x, self.y, self.z)
        self.pub_fiducial.publish(fiducial_msg)
        
        self.img = img_display

    # Checks if the contour is a square
    def is_square(self, cnt, epsilon):
        cnt_len = cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, epsilon * cnt_len, True)

        if len(cnt) != 4 or not cv2.isContourConvex(cnt):
            return (cnt, False)
        else:
            cnt = cnt.reshape(-1, 2)
            max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)]) 

            return (cnt, max_cos < 0.1)


    # On click listener function for selecting an object in the image
    def draw_circle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.mode == 3:
            roi_corners.append((x,y))

    # Calculates the angle between two points (helper function for 'find_squares()')
    def angle_cos(self, p0, p1, p2):
        d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
        return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

if __name__ == '__main__':
    try:
        var = Vision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
