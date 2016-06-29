import rospy
from Missions import *
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from mavros_msgs.msg import BatteryStatus, State, OverrideRCIn, Waypoint, RCIn
from multirotors.msg import stick_cmd, toggle_cmd
# from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from cv_bridge import CvBridge, CvBridgeError
from mission_planning.mission_planner import *

joystick = {'arm': 2, 'disarm': 3, 'failsafe': 0, 'auto': 4, 'manual': 5, 'x': 0, 'y': 1, 'z': 3, 'yaw': 2}
xbox     = {'arm': 0, 'disarm': 1, 'failsafe': 8, 'auto': 2, 'manual': 3, 'x': 3, 'y': 4, 'z': 1, 'yaw': 0}
rcsim    = {'arm': 2, 'disarm': 2, 'failsafe': 8, 'auto': 1, 'manual': 3, 'x': 0, 'y': 1, 'z': 2, 'yaw': 4}
rcsim_lim = [[-.707,.653],[-.587,.598],[-.620,.598],[-1,1],[-.772,.631],[-1,1]]

ctrl = joystick # Set this variable to the joystick you are currently using

modes = {'stabilize':'0', 'alt_hold':'2', 'auto':'3', 'loiter':'5', 'guided':'4', 'rtl':'6', 'land':'9'}

# This is a helper class that encodes all of the callback functions for a drone
class Drone(Missions):
    def __init__(self):
        # Joystick Variables
        self.buttons = [0]*10 #initialize all 8 buttons as 0
        self.axes = [0]*8 #initialize all 8 axes as 0
        self.joy_cmds = [0]*8
        self.mode = 0
        self.armed = False
        self.flight_mode = ''
        self.voltage = 0
        self.current = 0
        self.battery_remaining = 0
        self.just_armed = False

        namespace='drone'

        # ROS publishers
        self.pub_rc = rospy.Publisher('/%s/rc/override' %namespace, OverrideRCIn, queue_size=10)

        # ROS subscribers
        self.sub_state = rospy.Subscriber('/%s/state' %namespace, State, self.state_callback)
        self.sub_battery = rospy.Subscriber('/%s/battery' %namespace, BatteryStatus, self.battery_callback)
        
        self.sub_stick_cmds = rospy.Subscriber('/stick_cmds', stick_cmd, self.stick_cmd_callback)
        self.sub_toggle_cmds = rospy.Subscriber('/toggle_cmds', toggle_cmd, self.toggle_cmd_callback)

        # ROS services
        self.srv_arm = rospy.ServiceProxy('/%s/cmd/arming' %namespace, CommandBool)
        self.srv_mode = rospy.ServiceProxy('/%s/set_mode' %namespace, SetMode)

        super(Drone, self).__init__()

    """ Publishes to the 8 RC channels """
    def publish_rc(self, channels):
        """ publish rc channels to drone
            [roll, pitch, throttle, yaw, ch5(mode), 0, ch7, ch8]
        """
        channels[5] = 0
        commands = OverrideRCIn(channels=channels)
        self.pub_rc.publish(commands)

    """ Joystick control (can be overriden) """
    def fly_joystick(self):
        channels = self.joy_cmds
        self.publish_rc(channels)

    """ Various callback functions for each ROS topic """
    def stick_cmd_callback(self, data):
        """ callback for input stick commands 
            (roll, pitch, yaw, throttle) 
        """
        if self.just_armed:
            throttle = 1000
            if abs(data.throttle) > 0.1:
                self.just_armed = False
        else:
            throttle = data.throttle
        self.joy_cmds = [data.roll, data.pitch, throttle, data.yaw, 0, 0, 0, 0]


    def toggle_cmd_callback(self, data):
        """ callback for toggle-able commands
            (arm, disarm, mode, land, rtl, ...)
        """
        if data.arm and not self.armed:
            self.arm()
        if data.disarm and self.armed and not self.just_armed:
            self.disarm()
        if data.rtl:
            self.RTL()
        if data.land:
            self.land()
        if data.fiducial:
            self.fiducial()
        if data.planner:
            planner = Map_Planner(self)
        if data.takeoff:
            pass
        if data.mode:
            self.mode = data.mode
            self.srv_mode(0, modes[self.mode])
            print 'switched to %s' %data.mode
        

    def scale_axes(self, limits):
        for ch in range(len(self.axes)):
            if self.axes[ch] > 0:
                self.axes[ch] *= 1./limits[ch][1]
            else:
                self.axes[ch] *= -1./limits[ch][0]

    def state_callback(self, data):
        self.armed = data.armed
        self.flight_mode = data.mode

    def battery_callback(self, data):
        self.voltage = data.voltage
        self.current = data.current
        self.battery_remaining = data.remaining

    def RTL(self):
        self.mode = 'rtl'
        self.srv_mode(0, modes[self.mode])
        print 'returning to launch'

    def land(self):
        self.mode = 'land'
        self.srv_mode(0, modes[self.mode])
        print 'landing'

    def fiducial(self):
        self.mode = 'fiducial'
        self.srv_mode(0, modes['alt_hold'])
        print 'centering'

    def arm(self):
        self.publish_rc([1500, 1500, 1000, 1500, 0, 0, 0, 0])
        self.srv_arm(True)
        #self.just_armed = True
        self.armed = True
        print "Arming drone"

    def disarm(self):
        self.srv_arm(False)
        print "Disarming drone"
