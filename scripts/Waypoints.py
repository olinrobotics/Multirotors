""" stores all of the waypoint related methods
for the drone class """

import rospy
from mavros_msgs.msg import Waypoint
from mavros.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent

DEFAULT_ALT = 15

class Waypoints(object):
    """ helper class for managing waypoints on drone """
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = 0.0

        self.srv_wp_push = rospy.ServiceProxy('/drone/mission/push', WaypointPush)
        self.srv_wp_clear = rospy.ServiceProxy('/drone/mission/clear', WaypointClear)
        #self.srv_wp_goto = rospy.ServiceProxy('/drone/mission/goto', WaypointGOTO)
        self.srv_set_current = rospy.ServiceProxy('/drone/mission/set_current', WaypointSetCurrent)

    def gps_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def altitude_callback(self, data):
        self.altitude = data

    def make_global_waypoint(self, lat, lon, alt=DEFAULT_ALT, hold=5):
        waypoint = Waypoint()

        waypoint.frame = 3
        waypoint.command = 16
        waypoint.is_current = 0
        waypoint.autocontinue = False
        waypoint.param1 = hold #hold time
        waypoint.param2 = 2
        waypoint.param3 = 0
        waypoint.param4 = 0
        waypoint.x_lat = lat
        waypoint.y_long = lon
        waypoint.z_alt = alt

        return waypoint

    def push_waypoints(self):
        res = self.srv_wp_push(self.waypoints)
        success = res.success
        if success:
            print "Pushed waypoints"
        else:
            print "Failed to push waypoints"
        return success

    def clear_waypoints(self):
        res = self.srv_wp_clear()
        success = res.success

        if success:
            self.waypoints = []
            print "Cleared waypoints"
        else:
            print "Failed to clear waypoints"

    def add_waypoints(self, coordinates):
        '''adds waypoints to the end of the current mission'''
        new_waypoints = [self.make_global_waypoint(lat, lon) for [lat, lon] in coordinates]
        self.waypoints.extend(new_waypoints)
        #self.continue_mission() #don't land at end of mission
        self.push_waypoints()

    def reset_waypoints(self, waypoints):
        self.waypoints = waypoints
        #self.continue_mission()
        self.push_waypoints()
        self.restart_mission()