import rospy
from Missions import *
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from mavros_msgs.msg import BatteryStatus, State, OverrideRCIn, Waypoint
# from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from cv_bridge import CvBridge, CvBridgeError
from mission_planning.mission_planner import *

joystick = {'arm': 2, 'disarm': 3, 'failsafe': 0, 'auto': 4, 'manual': 5, 'x': 0, 'y': 1, 'z': 3, 'yaw': 2}
xbox     = {'arm': 0, 'disarm': 1, 'failsafe': 8, 'auto': 2, 'manual': 3, 'x': 3, 'y': 4, 'z': 1, 'yaw': 0}
rcsim    = {'arm': 2, 'disarm': 2, 'failsafe': 8, 'auto': 2, 'manual': 3, 'x': 0, 'y': 1, 'z': 2, 'yaw': 4}
rcsim_lim = [[-.707,.653],[-.587,.598],[-.620,.598],[-1,1],[-.772,.631],[-1,1]]

ctrl = rcsim # Set this variable to the joystick you are currently using

# This is a helper class that encodes all of the callback functions for a drone
class Drone(Missions):
    def __init__(self):
        # Joystick Variables
        self.buttons = [0]*10 #initialize all 8 buttons as 0
        self.axes = [0]*8 #initialize all 8 axes as 0
        self.mode = 0
        self.armed = False
        self.flight_mode = ''
        self.voltage = 0
        self.current = 0
        self.battery_remaining = 0
        self.old_z = 1000
        self.just_armed = False
        self.planner_up = False

        # ROS publishers
        self.pub_rc = rospy.Publisher('/drone/rc/override', OverrideRCIn, queue_size=10)

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.sub_state = rospy.Subscriber('/drone/state', State, self.state_callback)
        self.sub_battery = rospy.Subscriber('/drone/battery', BatteryStatus, self.battery_callback)

        # ROS services
        self.srv_arm = rospy.ServiceProxy('/drone/cmd/arming', CommandBool)
        self.srv_mode = rospy.ServiceProxy('/drone/set_mode', SetMode)

        super(Drone, self).__init__()

    """ Publishes to the 8 RC channels """
    def publish_rc(self, channels):
        commands = OverrideRCIn(channels=channels)
        self.pub_rc.publish(commands)

    """ Joystick control (can be overriden) """
    def fly_joystick(self, x=0, y=0, z=0, yaw=0):
        if self.buttons:
            # Arm drone
            if self.buttons[ctrl['arm']] and not self.armed:
                self.publish_rc([1500, 1500, 1000, 1500, 0, 65535, 0, 0])
                self.srv_mode(0, '2')
                self.srv_arm(True)
                self.just_armed = True
                self.armed = True
                self.old_z = self.axes[1]
                print "Arming drone"

            # Disarm drone
            if self.buttons[ctrl['disarm']] and self.armed and not self.just_armed:
                self.srv_arm(False)
                print "Disarming drone"

            # this should probably be dealt with somewhere else, but I'm not sure how
            if self.buttons[4] and not self.planner_up:
                planner = Map_Planner(self)
                self.planner_up = True
            if not self.buttons[4] and self.planner_up:
                self.planner_up = False

        if self.armed:
            if not(x): x = 1500 - self.axes[ ctrl['x'] ] * 300
            if not(y): y = 1500 - self.axes[ ctrl['y'] ] * 300
            if not(z): z = 1500 + self.axes[ ctrl['z'] ] * 500
            if not(yaw): yaw = 1500 - self.axes[ ctrl['yaw'] ] * 500

            if self.just_armed:
                z = 1000
                if abs(self.axes[ ctrl['z'] ]) > 0.1:
                    self.just_armed = False

            if z < 1300:
                z = 1000
            elif z > 1450 and z < 1550:
            	z = 1500

            if abs(x - 1500) < 50:
                x = 1500
            if abs(y - 1500) < 50:
                y = 1500

            channels = [x, y, z, yaw, 0, 0, 1250, 0]
            self.publish_rc(channels)


    """ Various callback functions for each ROS topic """
    def joy_callback(self, data):
        self.axes = list(data.axes)
        self.buttons = data.buttons
        if ctrl == rcsim:
            self.scale_axes(rcsim_lim)

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
        self.mode = 'RTL'
        self.srv_mode(0, '6')
        print 'returning to launch'