""" stores all of the mission related methods 
for the drone class including geofencing functions 
and guided mode functins """

import rospy
from Waypoints import *
from Guided import *

class Missions(Guided, Waypoints):
    """ helper class containing functions to create and control missions on the drone """
    def __init__(self):
        self.waypoints = [] #initialize internal waypoint list
        self.BOUNDEDFLIGHT = False #for setting polygon flight boundaries
        self.bounds = [] #polygon corners

        super(Missions, self).__init__()
        Waypoints.__init__(self)

    def continue_mission(self):
        ''' make drone hold position forever at end of mission instead of land 
            sets a new waypoint in the same location as the last waypint with
            the mode set to loiter unlimited (17)
        '''
        final_lat = self.waypoints[-1].x_lat #get final waypoint's location
        final_lon = self.waypoints[-1].y_long
        hold_pos = self.make_global_waypoint(final_lat, final_lon) #make new waypoint
        hold_pos.command = 17 #set mode to loiter unlimited
        self.waypoints.append(hold_pos)

    def end_with_rtl(self):
        """ add RTL to end of mission """
        rtl = Waypoint()
        rtl.command = 20
        self.waypoints.append(rtl)

    def start_with_takeoff(self):
        """ add takeoff command to beginning of mission """
        start = self.make_global_waypoint(0, 0)
        takeoff = Waypoint()
        takeoff.command = 22
        takeoff.z_alt = DEFAULT_ALT
        self.waypoints.insert(0, takeoff)
        self.waypoints.insert(0, start)

    def save_mission_to_file(self, filename='last_mission'):
        """ saves mission to xml file (default 'last_mission') 
            uses format from mission planner
        """
        root = ET.Element('missions')
        mission = ET.SubElement(root, 'mission', id='0')
        for (i, point) in enumerate(self.waypoints):
            waypoint = ET.SubElement(mission, 'waypoint', num=str(i))
            frame = ET.SubElement(waypoint, 'frame').text = str(point.frame)
            is_current = ET.SubElement(waypoint, 'is_current').text = str(point.is_current)
            autocontinue = ET.SubElement(waypoint, 'autocontinue').text = str(point.autocontinue)
            command = ET.SubElement(waypoint, 'command').text = str(point.command)
            param1 = ET.SubElement(waypoint, 'param1').text = str(point.param1)
            param2 = ET.SubElement(waypoint, 'param2').text = str(point.param2)
            param3 = ET.SubElement(waypoint, 'param3').text = str(point.param3)
            param4 = ET.SubElement(waypoint, 'param4').text = str(point.param4)
            lat = ET.SubElement(waypoint, 'latitude').text = str(point.x_lat)
            lon = ET.SubElement(waypoint, 'longitude').text = str(point.y_long)
            alt = ET.SubElement(waypoint, 'altitude').text = str(point.z_alt)
        tree = ET.ElementTree(root)
        package_path = rospkg.RosPack().get_path('drone_control')
        tree.write(package_path + '/scripts/%s.xml' %filename)
        if filename == 'last_mission':
            print 'missoin saved: copy last_mission.xml to new file to keep'
        else:
            print 'mission saved to %s.xml' %filename

    def begin_mission(self):
        """ start mission on drone from armed """
        self.mode = 'auto'
        self.srv_mode(0, '3')
        rc_msg = OverrideRCIn()
        (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3]) = (1500, 1500, 1250, 1500) 
        self.pub_rc[0].publish(rc_msg)
        print "Beginning mission"

    def end_mission(self):
        """ quit mission by shifting to manual mode """
        self.mode = 'manual'
        self.srv_mode(0, '5')
        print "Ending mission"

    def restart_mission(self):
        """ restart mission at start """
        success = True
        self.srv_set_current(0)
        print "restarted mission"

    def check_bounds(self):
        """ checks if drone is in the polygon boundary """
        if self.BOUNDEDFLIGHT and self.armed and self.mode != 'RTL':
            pos = [self.latitude, self.longitude]
            if pos != [0, 0]:
                if not InPoly.isInPoly(self.bounds, pos):
                    self.RTL()