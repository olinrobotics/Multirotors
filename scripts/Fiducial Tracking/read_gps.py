'''
    OLD CODE

    Reads data from an Adafruit GPS Module (location of the fiducial)
'''


#!/usr/bin/env python
import rospy
import serial
import math

from geometry_msgs.msg import Point

class PlatformGPS():
    def __init__(self):
        rospy.init_node('platform')

        self.pub_gps = rospy.Publisher('platform_gps', Point, queue_size=10)
        
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.get_GPS()
            r.sleep()

    def deg_min_to_deg_dec(self, raw): 
        '''
        convert coordinate from degrees, minutes to decimal degrees
        '''
        deg = 100*math.floor(raw/100)
        mins = raw-deg
        final = deg + (mins/0.6)

        return final
    
    def get_GPS(self):
        '''
        outputs a single reading of latitue, longitude in decimal degrees
        '''
        ser = serial.Serial('/dev/ttyUSB1', 9600)
        while True:
            line = ser.readline()
            data = line.split(',')
            if data[0] == '$GPRMC': #find the right line
                coor = Point()

                if data[2] == 'A': #check for good signal
                    lon_raw = float(data[3]) #longitude in deg min
                    lat_raw = float(data[5]) #latitute in deg min
                    lon = self.deg_min_to_deg_dec(lon_raw) / 100
                    lat = self.deg_min_to_deg_dec(lat_raw) / 100
                    if data[4] == 'S': #make negative if needed
                        lon = -lon
                    if data[6] == 'W':
                        lat = -lat
                    
                    (coor.x, coor.y, coor.z) = (lat, lon, 1)
                else:
                    (coor.x, coor.y, coor.z) = (0, 0, 0)

                self.pub_gps.publish(coor) 

if __name__ == '__main__':
    try:
        var = PlatformGPS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


