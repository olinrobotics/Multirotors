import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import BatteryStatus, State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge, CvBridgeError

# This is a helper class that encodes all of the callback functions for a drone
class Drone():
    def __init__(self):
        # ROS state variables
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

    """ Joystick control """
    def fly_joystick(self):
        if self.buttons:
            # Arm drone
            if self.buttons[0] and not self.drone.armed:
                self.srv_mode(0, '2')
                self.srv_arm(True)
                self.just_armed = True
                self.old_z = self.axes[1]
                print "Arming drone"

            # Disarm drone
            if self.buttons[1]:
                self.srv_arm(False)
                print "Disarming drone"

        if self.drone.armed:
            x = 1500 - self.axes[3] * 300
            y = 1500 - self.axes[4] * 300
            yaw = 1500 - self.axes[0] * 200

            if self.just_armed:
                z = 1000
            else:
                z = 1500 + self.axes[1] * 500
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