""" stores all of the mission related methods 
for the drone class including geofencing functions 
and guided mode functins """

import rospy
from Waypoints import *

class Guided(Waypoints):
    def __init__(self):
    	self.guided_waypoints = []
        self.guided_index = 0

    def start_guided(self):
        self.mode = 'guided'
        self.srv_mode(0, '4')
        self.guided_index = 0
        print "guided mode started"

    def set_guided_waypoint(self, lat, lon, alt=DEFAULT_ALT):
        waypoint = self.make_global_waypoint(lat, lon, alt)
        waypoint.is_current = 2
        res = self.srv_wp_push([waypoint])
        if res.success:
            print "set waypoint"
        else:
            print "waypoint failed"

    def guided_function(self):
        if self.mode == 'guided':
            if self.guided_index >= len(self.guided_waypoints):
                self.RTL()
            else:
                waypoint = self.guided_waypoints[self.guided_index]
                self.set_guided_waypoint(waypoint[0], waypoint[1], waypoint[2])
                self.guided_index += 1
        else:
            self.start_guided()