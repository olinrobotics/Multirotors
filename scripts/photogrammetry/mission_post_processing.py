#!/usr/bin/env python
""" Code which reads images into a GUI for measurement following a mission """

#Maintainer: Victoria Preston
#Successfully used with: Python 2.7.6, OpenCV 2.4.8 and Numpy 1.8.2

#TODO Adjust scaling parameters to reflect meters better

import cv2, sys
from cv2 import cv
import numpy as np
import argparse
import numpy as np
import PIL.Image
import PIL.ImageTk
import Tkinter as tk
import copy
import glob


class ImageManager(object):
	""" This handles the actual image displays """
	def __init__(self):
		#set-up constants
		global FILENAME

		self.sensor_height_mm = 4.04
		self.sensor_width_mm = 5.37
		self.focal_length = 14.0 #should be measured more...going off of website
		self.foot_to_mm = 304.8

		#will want to change to read the file name
		major_alt = float(FILENAME.split('/')[1].split('.')[0].split('_')[0])
		minor_alt = float(FILENAME.split('/')[1].split('.')[0].split('_')[1])/100.
		self.altitude_m = major_alt+minor_alt
		print self.altitude_m

		self.distance_to_object_mm = self.altitude_m*1000 - self.foot_to_mm

		#establish image info
		self.base_image = self.get_image(FILENAME)
		self.img = copy.copy(self.base_image) #so we can reset things if the need be
		self.im_height_pix = 1080.0
		self.im_width_pix = 1920.0

		self.points = []
		self.saved_mult = 0

	def get_image(self, image):
		return cv2.imread(image)

	def clear_everything(self):
		self.points = []
		self.img = copy.copy(self.base_image)

	def add_point(self, event):
		#grab the coordinates
		x = event.x
		y = event.y

		#plot them
		self.plot_point(x, y)

		#return them
		return x, y

	def plot_point(self, x, y):
		self.points.append([x,y])

	def calculate_distance(self):
		pixel_distance_height = np.sqrt((self.points[0][1] - self.points[1][1])**2)
		pixel_distance_width = np.sqrt((self.points[0][0] - self.points[1][0])**2)
		
		pixel_to_object_height = pixel_distance_height * ((self.distance_to_object_mm*self.sensor_height_mm) / (self.focal_length*self.im_height_pix))
		pixel_to_object_width = pixel_distance_width * ((self.distance_to_object_mm*self.sensor_width_mm) / (self.focal_length*self.im_width_pix))

		true_distance = np.sqrt((pixel_to_object_height**2 + pixel_to_object_width**2)) * 0.0393701
		print 'width, pix ', pixel_distance_width 
		print 'height, pix ', pixel_distance_height 
		print 'true distance, in ', true_distance 

		return true_distance 

	def save_all(self):
		global FILENAME
		print 'Saving your image!'
		save_name = 'processed_images/processed_%s_%s' % (str(self.saved_mult),FILENAME.split('/')[1])
		print save_name
		self.saved_mult += 1
		cv2.imwrite(save_name, self.img)
		print 'Saved!'


class MeasurementGui():
	""" This creates the Tkinter window and manages the interface """

	def __init__(self):

		#set all the initial values
		self.distance_measured = 0

		#establish the tkinter window and any special protocol
		self.root = tk.Tk() 
		setattr(self.root, 'quit_flag', False)
		self.root.protocol('WM_DELETE_WINDOW', self.set_quit_flag)

		#create the button console
		self.buttons()

		#create distance read out
		self.distance_text = tk.StringVar()
		self.distance_text.set('distance measured: %d' %self.distance_measured)
		self.labels() 

		#let's actually display the image we want to process
		self.display_image()

		#set the geometry of it all
		self.root.geometry('1080x720+500+200') #currently set to less than full size of the image

		#run everything
		self.root.mainloop()

	def set_quit_flag(self):
		#just a simple toggle
		self.root.quit_flag = True

	def loop(self):
		""" main loop, allows the quitting and refreshing to happen """
		if self.root.quit_flag:
			#quit the whole shehbang if you need to
			print 'destroying'
			self.root.destroy() 
		else:
			#otherwise, keep updating everything
			self.update()
			self.root.after(10, func=self.loop)

	def display_image(self):
		""" creates the label object so that the image can actually be displayed """
		
		#create the label
		self.image_label = tk.Label(self.root)
		
		#pack the label, which allows for widgets to be displayed
		self.image_label.pack()
		
		#get the image
		self.image = ImageManager()

		#refresh everything
		self.root.after(0, func=self.loop)

		#allow for clicking to add points!
		self.image_label.bind('<Button-1>', self.add_point)

	def add_point(self, event):
		self.image.add_point(event)

	def buttons(self):
		""" creates a console for making measurements """

		#create a save measurement button
		save_measurement_button = tk.Button(self.root, text='Save Measurement',command=self.save_measurement)
		save_measurement_button.pack(side=tk.TOP, padx=5, pady=5)

		#create a clear button
		clear_button = tk.Button(self.root, text='Clear', command=self.clear_measurement)
		clear_button.pack(side=tk.TOP, padx=5, pady=5)

	def labels(self):
		dist_label = tk.Label(self.root, textvariable=self.distance_text,padx=10)
		dist_label.pack(side=tk.TOP)

	def save_measurement(self):
		self.image.save_all()

	def clear_measurement(self):
		""" essentially resets everything without saving """
		self.image.clear_everything()
		self.distance_measured = 0
		self.distance_text.set('distance measured: %d' %self.distance_measured)

	def update(self):
		""" handles gathering the images and other elements that need to be displayed onto the gui """
		points = self.image.points
		for i in range(len(points)):
			if i > 0:
				cv2.line(self.image.img,(points[i-1][0],points[i-1][1]),(points[i][0],points[i][1]),(0,0,255))
				cv2.circle(self.image.img, (points[i][0],points[i][1]),5,(0,0,255),-1)
				self.distance_measured = self.image.calculate_distance()
				self.distance_text.set('distance measured: %d' %self.distance_measured)
			else:
				cv2.circle(self.image.img,(points[i][0],points[i][1]),5,(0,0,255),-1)
		#take the image and make it compatible with Tkinter
		rgb_img = cv2.cvtColor(self.image.img, cv2.COLOR_BGR2RGB)
		pil_image = PIL.Image.fromarray(rgb_img)
		tk_image = PIL.ImageTk.PhotoImage(image=pil_image)

		#put the image into the 'label' which does the display thing
		self.image_label.configure(image=tk_image)

		#save a cache of the images for later
		self.image_label._image_cache = tk_image

		#update everything
		self.root.update()


# def draw_line(points_list):
#     sensor_height_mm = 4.04 
#     sensor_width_mm = 5.37

#     focal_length_mm = 1289.0

#     foot_offset_mm = 304.8
#     altitude_ft = 30.0
#     distance_to_object_mm = altitude_ft * 302.8 - foot_offset_mm #will want to replace with altitude data
	
#     im_height_pix = 1080.0
#     im_width_pix = 1920.0
	
#     cv2.line(img,points_list[0],points_list[1],(0,0,255))

#     pixel_distance_height = np.sqrt((points_list[0][1] - points_list[1][1])**2)
#     pixel_distance_width = np.sqrt((points_list[0][0] - points_list[1][0])**2)
	
#     pixel_to_object_height = pixel_distance_height * (distance_to_object_mm / focal_length_mm )
#     pixel_to_object_width = pixel_distance_width * (distance_to_object_mm / focal_length_mm)


#     true_distance = np.sqrt((pixel_to_object_height**2 + pixel_to_object_width**2)) 
#     print 'width, pix ', pixel_distance_width 
#     print 'height, pix ', pixel_distance_height 
#     print 'true distance, in ', true_distance * 0.00328084 * 12.0

if __name__ == '__main__':
	global DIRECTORY
	global FILENAME
	#establish a single global for file to process
	DIRECTORY = 'images_for_processing/*'
	for f in glob.iglob(DIRECTORY):
		print 'filename', f
		FILENAME = f
		MeasurementGui()
