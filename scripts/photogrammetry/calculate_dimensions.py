#!/usr/bin/env python
""" For any given image with a calculation constant, report back the dimensions of the object. Executes a point and click GUI by which to select the points of interest."""

#Maintainer: Victoria Preston
#Successfully used with: Python 2.7.6, OpenCV 2.4.8 and Numpy 1.8.2

import cv2, sys
from cv2 import cv
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
        height = video.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
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
                cv2.imwrite('Image_Selected' + str(collected_images) + '.png', image)
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
	
#click on the set of points to measure
#get a reading

if name == '__main__':
	#pull in the video you would like to select the image to process from
	filename = 'Videos/normal_lens_calib.mp4'
	list_images = ['image_to_process.png']
	#process input
	list_images = collect_image(filename) #if video
	process_image(list_images[0])