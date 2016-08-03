import rospy
from Missions import *
from std_msgs.msg import String
from mavros_msgs.msg import BatteryStatus, State, OverrideRCIn, Waypoint, RCIn
from multirotors.msg import stick_cmd, toggle_cmd
# from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from cv_bridge import CvBridge, CvBridgeError
from mission_planning.mission_planner import *
import time

modes = {'stabilize':'0', 'alt_hold':'2', 'auto':'3', 'loiter':'5', 'guided':'4', 'rtl':'6', 'land':'9'}

# This is a helper class that encodes all of the callback functions for a drone
class Drone(Missions):
    def __init__(self):
        # Joystick Variables
        self.joy_cmds = [0]*8 #initialize joystic commands to 0
        self.mode = 0 #initialize mode to stabilize
        self.armed = False #initialize as disarmed
        self.flight_mode = '' #reported flight mode (unset until message is received)
        self.voltage = 0 #reported voltage initialize variable
        self.current = 0 #reported current initialize variable
        self.battery_remaining = 0 #reported battery remaining initialize variable
        self.just_armed = False #used for arming debouncing

        namespace='drone' #namespace for all drone rostopics

        # ROS publishers
        self.pub_rc = rospy.Publisher('/%s/rc/override' %namespace, OverrideRCIn, queue_size=10)
        #publish to rc/override to control drone

        # ROS subscribers
        self.sub_state = rospy.Subscriber('/%s/state' %namespace, State, self.state_callback)
        #listen to drone's state
        self.sub_battery = rospy.Subscriber('/%s/battery' %namespace, BatteryStatus, self.battery_callback)
        #listen to battery info

        self.sub_stick_cmds = rospy.Subscriber('/stick_cmds', stick_cmd, self.stick_cmd_callback)
        self.sub_toggle_cmds = rospy.Subscriber('/toggle_cmds', toggle_cmd, self.toggle_cmd_callback)
        #listen to user inputs: joystick sytle inputs and toggle style inputs

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
            #self.find_target()
        if data.planner:
            planner = Map_Planner(self)
        if data.takeoff:
            pass
        if data.mode:
            self.mode = data.mode
            self.srv_mode(0, modes[self.mode])
            print 'switched to %s' %data.mode

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
        self.srv_mode(0, modes['loiter'])
        print 'centering'

    def find_target(self):
        self.srv_mode(0, modes['guided'])
        time.sleep(2)
        self.mode = 'find_target'
        print 'returning to target'

    def arm(self):
        self.publish_rc([1500, 1500, 1000, 1500, 0, 0, 0, 0])
        self.srv_arm(True)
        #self.just_armed = True
        self.armed = True
        print "Arming drone"

    def disarm(self):
        self.srv_arm(False)
        print "Disarming drone"
