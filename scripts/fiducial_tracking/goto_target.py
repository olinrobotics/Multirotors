import rospy
import serial
from math import *

ALT = 15
	
class GotoTarget():
	def __init__(self, drone):
		self.drone = drone
		self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
		self.last_coords = [1, 1]
		self.saw_fiducial = False

	def dec_degree(self, coordinate, direction):
		""" input string of coordinate and direction from nmea message,
		output decimal degree as float """
		coord = float(coordinate)/100.
		whole = floor(coord)
		decimal = (coord%1)/.6
		final = whole+decimal
		#final = float(coordinate)
		if direction in ('S', 'W'):
			final*=-1
		return final

	def parse(self, line):
		data = line.split(',')
		#print 'full', line
		if line and data[0] == '$GPGGA':
			try:
				lat = self.dec_degree(data[2], data[3])
				lon = self.dec_degree(data[4], data[5])
			except:
				print 'no coords'
				return None
			return [lat, lon]

	def run(self):
		line = self.ser.readline()
		coords = self.parse(line)
		#print self.drone.mode, '\n'
		if coords and self.drone.mode == 'find_target':
			#print 'here'
			if abs(coords[0]-self.last_coords[0]) > 5*10**-6 or \
			   abs(coords[1]-self.last_coords[1]) > 5*10**-6:
				self.drone.set_guided_waypoint(coords[0], coords[1], ALT)
				print 'set waypoint'
				self.last_coords = coords

	def finished(self):
		return self.saw_fiducial
