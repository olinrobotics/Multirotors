import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import BatteryStatus, State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge, CvBridgeError

# This is a helper class that encodes all of the callback functions for a drone
class Drone():
    def __init__(self):
        # ROS state variables
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        self.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        self.mode = 0
        self.armed = False
        self.flight_mode = ''
        self.voltage = 0
        self.current = 0
        self.battery_remaining = 0
        self.latitude = None
        self.longitude = None
        self.altitude = 0.0
        self.old_z = 1000

        self.init_mission_vars()

        # ROS publishers
        self.pub_rc = rospy.Publisher('/rc/override', OverrideRCIn, queue_size=10)

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.sub_state = rospy.Subscriber('/state', State, self.state_callback)
        self.sub_battery = rospy.Subscriber('/battery', BatteryStatus, self.battery_callback)

        # ROS services
        self.srv_arm = rospy.ServiceProxy('/cmd/arming', CommandBool)
        self.srv_mode = rospy.ServiceProxy('/set_mode', SetMode)

    """ Publishes to the 8 RC channels """
    def publish_rc(self, channels):
        commands = OverrideRCIN(channels=channels)
        self.pub_rc.publish(commands)

    """ Joystick control (can be overriden) """
    def fly_joystick(self, x=0, y=0, z=0, yaw=0):
        if self.buttons:
            # Arm drone
            if self.buttons[0] and not self.armed:
                self.srv_mode(0, '2')
                self.srv_arm(True)
                self.just_armed = True
                self.armed = True
                self.old_z = self.axes[1]
                print "Arming drone"

            # Disarm drone
            if self.buttons[1]:
                self.srv_arm(False)
                print "Disarming drone"

        if self.armed:
            # Checks if the joystick commands are being overridden
            x   = x   if x != 0   else 1500 - self.axes[3] * 300
            y   = y   if y != 0   else 1500 - self.axes[4] * 300
            yaw = yaw if yaw != 0 else 1500 - self.axes[0] * 200

            if self.just_armed:
                z = 1000
            else:
                z = z if z != 0 else 1500 + self.axes[1] * 500
                if z < 1150:
                    z = 1000

            if self.axes[1] != self.old_z:
                self.just_armed = False

            channels = [x, y, z, yaw, 0, 0, 1250, 0]
            self.publish_rc(channels)



    """ Various callback functions for each ROS topic """
    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons

    def state_callback(self, data):
        self.armed = data.armed
        self.flight_mode = data.mode

    def battery_callback(self, data):
        self.voltage = data.voltage
        self.current = data.current
        self.battery_remaining = data.remaining

    def gps_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def altitude_callback(self, data):
        self.altitude = data



    """ 
        mission building functions:

        Make waypoints, modify current mission to start with takeoff
        or end by either returning to launch or holding in the last position
        And save current mission to a file for future use.
    """
    def init_mission_vars(self):
        self.waypoints = []
        self.guided_waypoints = []
        self.guided_index = 0
        self.BOUNDEDFLIGHT = False
        self.bounds = []

    def continue_mission(self):
        ''' make drone hold position forever at end of mission instead of land '''
        final_lat = self.waypoints[-1].x_lat
        final_lon = self.waypoints[-1].y_long
        hold_pos = self.make_global_waypoint(final_lat, final_lon)
        hold_pos.command = 17
        self.waypoints.append(hold_pos)

    def end_with_rtl(self):
        rtl = Waypoint()
        rtl.command = 20
        self.waypoints.append(rtl)

    def start_with_takeoff(self):
        start = self.make_global_waypoint(0, 0)
        takeoff = Waypoint()
        takeoff.command = 22
        takeoff.z_alt = DEFAULT_ALT
        self.waypoints.insert(0, takeoff)
        self.waypoints.insert(0, start)

    def make_global_waypoint(self, lat, lon, alt=DEFAULT_ALT, hold=5):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drone_control')
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

    def save_mission_to_file(self, filename='last_mission'):
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

    def restart_mission(self):
        success = True
        for i in xrange(self.num_drones):
            self.srv_set_current[i](0)
        print "restarted mission"



    """ 
        Drone Action Functions:

        Change flight modes (mission start, end or RTL)
        cycle through guided mode waypoints
        modify waypoints on drone
    """
    def start_guided(self):
        self.mode[0] = 'guided'
        self.srv_mode[0](0, '4')
        self.guided_index = 0
        print "guided mode started"

    def set_guided_waypoint(self, lat, lon, alt=DEFAULT_ALT):
        waypoint = self.make_global_waypoint(lat, lon, alt)
        waypoint.is_current = 2
        res = self.srv_wp_push[0]([waypoint])
        if res.success:
            print "set waypoint"
        else:
            print "waypoint failed"

    def guided_function(self):
        if self.mode[0] == 'guided':
            if self.guided_index >= len(self.guided_waypoints):
                self.RTL()
            else:
                waypoint = self.guided_waypoints[self.guided_index]
                self.set_guided_waypoint(waypoint[0], waypoint[1], waypoint[2])
                self.guided_index += 1
        else:
            self.start_guided()

    def check_bounds(self):
        if self.BOUNDEDFLIGHT and self.armed and self.mode[0] != 'RTL':
            pos = [self.latitude, self.longitude]
            if pos != [0, 0]:
                if not InPoly.isInPoly(self.bounds, pos):
                    self.RTL()


    def RTL(self):
        self.mode[0] = 'RTL'
        self.srv_mode[0](0, '6')
        print 'returning to launch'

    def push_waypoints(self):
        success = True
        for i in xrange(self.num_drones):
            res = self.srv_wp_push[i](self.waypoints)
            success &= res.success
        if success:
            print "Pushed waypoints"
        else:
            print "Failed to push waypoints"
        return success

    def clear_waypoints(self):
        success = True
        for i in xrange(self.num_drones):
            res = self.srv_wp_clear[i]()
            success &= res.success

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

    def begin_mission(self):
        self.mode[0] = 'auto'
        self.srv_mode[0](0, '3')
        rc_msg = OverrideRCIn()
        (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3], rc_msg.channels[4]) = (1500, 1500, 1250, 1500, 1400) 
        self.pub_rc[0].publish(rc_msg)
        print "Beginning mission"

    def end_mission(self):
        self.mode[0] = 'manual'
        self.srv_mode[0](0, '5')
        print "Ending mission"
