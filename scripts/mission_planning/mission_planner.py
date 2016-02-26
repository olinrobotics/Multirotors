import rospy
import tf
from ar_pose.msg import ARMarkers

from drone import *

CONF_THRESHOLD = 70

class Map_Planner():
    def __init__(self, drone):
        self.drone = drone
        self.google_waypoints = []
        self.do_takeoff_start = True
        self.do_rtl_end = True
        self.do_hold_forever_end = False

    #map gui functions
    def map_callback(self, data):
        self.google_waypoints = [[point.x, point.y, point.z] for point in data.waypoint.points]
        self.do_takeoff_start = data.takeoff
        self.do_rtl_end = data.rtl
        self.do_hold_forever_end = data.hold_forever
        if data.dest == 'mission':
            self.set_mission_to_map()
        if data.dest == 'guided':
            self.set_guided_to_map()
        if data.dest == 'bounds':
            self.set_bounds_to_map()

    def launch_map(self):
        rospack = rospkg.RosPack()
        os.system('gnome-terminal -x %s/scripts/start_map.sh' %rospack.get_path('drone_control'))

    def set_guided_to_map(self):
        self.drone.guided_waypoints = self.google_waypoints
        self.drone.guided_index = 0
        print "guided points reset"

    def set_mission_to_map(self):
        self.drone.waypoints = [self.make_global_waypoint(lat, lon, alt) for [lat, lon, alt] in self.google_waypoints]
        if self.do_takeoff_start:
            self.drone.start_with_takeoff()
        if self.do_rtl_end:
            self.drone.end_with_rtl()
        if self.do_hold_forever_end:
            self.drone.continue_mission()
        self.push_waypoints()

    def set_bounds_to_map(self):
        #set the boundaries of a bounded flight (z is unused)
        print 'setting bounds'
        self.drone.BOUNDEDFLIGHT = True
        self.drone.bounds = [[x, y] for [x, y, z] in self.google_waypoints]

if __name__ == '__main__':
    try:
        var = FiducialFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass