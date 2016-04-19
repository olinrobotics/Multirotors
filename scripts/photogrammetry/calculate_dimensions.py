#!/usr/bin/env python
""" For any given image with a calculation constant, report back the dimensions of the object. Executes a point and click GUI by which to select the points of interest."""

#Maintainer: Victoria Preston
#Successfully used with: Python 2.7.6, OpenCV 2.4.8 and Numpy 1.8.2

#TODO: Make this a class, data flow is just so much better
#TODO: Want to read in from an altitude node for each image capture in order to make an appropriate rough calculation. Could also make a selected fixed distance command for photogrammetry missions

import cv2, sys
from cv2 import cv
import numpy as np
import argparse
import numpy as np

#select an image from a video to process, if given a video to process
def collect_image(filename, num_images=1):
    """Collect image(s) to eventually measure. Adapted from Chris Rillahan"""
    print 'LOADING YOUR VIDEO'
    print 'Use the SPACEBAR to select the image that you would like to process. Press ESC to quit.'

    #Load the file given to the function
    video = cv2.VideoCapture(filename)
    list_images = []

    #make sure loaded properly
    if video.isOpened():
        #Collect metadata about the file.
        FPS = video.get(cv.CV_CAP_PROP_FPS)
        FrameDuration = 1/(FPS/1000)
        width = video.get(cv.CV_CAP_PROP_FRAME_WIDTH)
        print width
        height = video.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
        print height
        size = (int(width), int(height))
        total_frames = video.get(cv.CV_CAP_PROP_FRAME_COUNT)

        #Initializes the frame counter and collected_image counter
        current_frame = 0
        collected_images = 0

        #Video loop.  Press spacebar to collect images.  ESC terminates the function.
        while current_frame < total_frames:
            success, image = video.read()
            current_frame = video.get(cv.CV_CAP_PROP_POS_FRAMES)
            cv2.imshow('Video', image)
            k = cv2.waitKey(int(FrameDuration)) #may want to make slower?
            if collected_images == num_images: 
                break
            if k == 32:
                collected_images += 1
                cv2.imwrite(str(collected_images) + '.png', image)
                list_images.append(str(collected_images)+'.png')
                print(str(collected_images) + ' image(s) collected.')
            if k == 27:
                break
        #Clean up
        video.release()
        cv2.destroyAllWindows()
    else:
        print('Error: Could not load video')
        sys.exit()
    return list_images

def process_image(image):
	"""This opens a point and click GUI by which a user can select two points to measure between"""
	#camera constants and knowns
	image_size = (1920, 1080)
	height = 1 #in meters please
	global img
	img = cv2.imread(str(image))
	cv2.namedWindow('image')
	cv2.setMouseCallback('image',draw_points)

	while(1):
	    cv2.imshow('image',img)
	    k = cv2.waitKey(1) & 0xFF
	    if k == 27:
	        break

	cv2.destroyAllWindows()

def draw_points(event,x,y,flags,param):
    global choosing, points

    if event == cv2.EVENT_LBUTTONDOWN:
        choosing = True

    elif event == cv2.EVENT_LBUTTONUP:
        choosing = False
        cv2.circle(img,(x,y),5,(0,0,255),-1)
        points.append((x,y))
        if len(points) >= 2:
        	draw_line(points)
        	points.pop(0)

def draw_line(points_list):
    sensor_height_mm = 4.04 
    sensor_width_mm = 5.37

    focal_length_mm = 1289.0

    foot_offset_mm = 304.8
    altitude_ft = 30.0
    distance_to_object_mm = altitude_ft * 302.8 - foot_offset_mm #will want to replace with altitude data
    
    im_height_pix = 1080.0
    im_width_pix = 1920.0
    
    cv2.line(img,points_list[0],points_list[1],(0,0,255))

    pixel_distance_height = np.sqrt((points_list[0][1] - points_list[1][1])**2)
    pixel_distance_width = np.sqrt((points_list[0][0] - points_list[1][0])**2)
    
    # pixel_to_object_height = (pixel_distance_height*distance_to_object_mm*sensor_height_mm)/(focal_length_mm*im_height_pix)
    # pixel_to_object_width = (pixel_distance_width*distance_to_object_mm*sensor_width_mm)/(focal_length_mm*im_width_pix)

    pixel_to_object_height = pixel_distance_height * (distance_to_object_mm / focal_length_mm )
    pixel_to_object_width = pixel_distance_width * (distance_to_object_mm / focal_length_mm)


    true_distance = np.sqrt((pixel_to_object_height**2 + pixel_to_object_width**2)) 
    print 'width, pix ', pixel_distance_width 
    print 'height, pix ', pixel_distance_height 
    print 'true distance, in ', true_distance * 0.00328084 * 12.0

if __name__ == '__main__':
	#pull in the video you would like to select the image to process from
	filename = 'videos/GOPR0206.MP4'
	choosing = False # true if mouse is pressed
	points = []

	list_images = ['1.png']
	# #process input
	# list_images = collect_image(filename) #if video
	process_image(list_images[0])

	