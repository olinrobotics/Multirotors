#!/usr/bin/env python
import rospy
import roslib
import time
import math
roslib.load_manifest('mavros')

from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float64, UInt8
from sensor_msgs.msg import Joy, NavSatFix
from mavros.msg import BatteryStatus, State, OverrideRCIn, Waypoint, WaypointList
from mavros.srv import CommandBool, CommandHome, WaypointPush, WaypointClear, WaypointGOTO, SetMode

from drone import *
import mission_parser
import gps_tools

joystick = {'arm': 2, 'disarm': 3, 'failsafe': 0, 'auto': 4, 'manual': 5, 'x': 0, 'y': 1, 'z': 3, 'yaw': 2}
xbox     = {'arm': 0, 'disarm': 1, 'failsafe': 8, 'auto': 2, 'manual': 3, 'x': 3, 'y': 4, 'z': 1, 'yaw': 0}

ctrl = xbox # Set this variable to the joystick you are currently using

class Snotbot():
    ###
    # Initializes drone control variables
    ### 
    def __init__(self):
        # Initializes ROS node
        rospy.init_node('snotbot')

        # Joystick variables
        self.axes = []
        self.buttons = []
        self.controller = xbox

        # Drone variables 
        self.drone = Drone(0)  # Drone class contains valuable functions/variables

        # Landing platform variables
        self.platform_gps = (0, 0)

        # Vision variables
        self.fiducial = (0, 0, 0) 
        self.t0 = 0.
        self.t1 = 0.
        self.error_x_sum = 0.
        self.error_y_sum = 0.
        self.error_x_prev = 0.
        self.error_y_prev = 0.

        # Miscellaneous variables
        self.time_mode_started = 0    # Records when each mode in the finite state machine started 
        self.target_alt = 12
        self.failsafe = True 
        self.launched = False
        self.returning = False
        self.just_armed = True

        # ROS publishers
        self.pub_rc = rospy.Publisher('/drone/rc/override', OverrideRCIn, queue_size=10)
        self.pub_mode = rospy.Publisher('mode', UInt8, queue_size=10)

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.sub_vision = rospy.Subscriber('/vision', Point, self.vision_callback)
        self.sub_platform_gps = rospy.Subscriber('/platform_gps', Point, self.platform_gps_callback)

        self.sub_state = rospy.Subscriber('/drone/state', State, self.drone.state_callback)
        self.sub_battery = rospy.Subscriber('/drone/battery', BatteryStatus, self.drone.battery_callback)
        self.sub_drone_gps = rospy.Subscriber('/drone/gps/fix', NavSatFix, self.drone.gps_callback)
        self.sub_altitude = rospy.Subscriber('/drone/global_position/rel_alt', Float64, self.drone.altitude_callback)
        self.sub_waypoints = rospy.Subscriber('/drone/mission/waypoints', WaypointList, self.drone.waypoints_callback)

        # ROS services
        self.srv_arm = rospy.ServiceProxy('/drone/cmd/arming', CommandBool)
        self.srv_mode = rospy.ServiceProxy('/drone/set_mode', SetMode)
        self.srv_wp_push =  rospy.ServiceProxy('/drone/mission/push', WaypointPush)
        self.srv_wp_clear = rospy.ServiceProxy('/drone/mission/clear', WaypointClear)

        # ROS parameters
        self.outdoors = rospy.get_param('~outdoors')

        # Waits to process all parameters before launching 
        print "Waiting for drone to connect..."
        for i in reversed(range(5)):
            if rospy.is_shutdown():
                break
            print str(i+1) + "..."
            time.sleep(1)

        if not rospy.is_shutdown():
            print "Ready"

        # Main loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

        # Returns control to the radio when the main loop has died
        self.kill_command()

    ###
    # Retrieves joystick data
    ###
    def joy_callback(self, data):
        self.axes = list(data.axes)
        self.buttons = list(data.buttons)

        dead_zone = 0.1
        if ctrl == xbox:
            if abs(self.axes[ ctrl['x'] ]) < dead_zone:
                self.axes[ ctrl['x'] ] = 0.0
            if abs(self.axes[ ctrl['y'] ]) < dead_zone:
                self.axes[ ctrl['y'] ] = 0.0
            if abs(self.axes[ ctrl['z'] ]) < dead_zone:
                self.axes[ ctrl['z'] ] = 0.0
            if abs(self.axes[ ctrl['yaw'] ]) < dead_zone:
                self.axes[ ctrl['yaw'] ] = 0.0

    ###
    # Retrieves vision data
    ###
    def vision_callback(self, data):
       self.fiducial = (data.x, data.y, data.z)

    ###
    # Resets all important variables to default
    ###
    def reset_variables(self):
        self.launched = False
        self.returning = False
        self.just_armed = True

    ###
    # Retrieves GPS data from platform
    ###
    def platform_gps_callback(self, data):
        self.platform_gps = (data.x, data.y)

    ###
    # Ends joystick control
    ###
    def kill_command(self):
        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535 for i in range(8)] # Stops override
        self.pub_rc.publish(rc_msg)

    ########################
    # FINITE STATE MACHINE #
    ########################

    ###
    # Mode 1: arms the drone
    ###
    def arm(self):
        self.drone.z = 1000

        # Arming process
        if not self.drone.armed:
            self.srv_mode(0, '5') # Loiter
            self.srv_arm(True)

        # Waits a few seconds before switching to launch mode
        if millis() - self.time_mode_started > 3000:
            print "Launching drone"
            self.time_mode_started = millis()
            self.drone.mode = 2

    ###
    # Mode 2: launches the drone to a certain altitude
    ###
    def launch(self):
        self.drone.z = 1250

        # Launches the drone at the current location
        if not self.launched:
            waypoints = mission_parser.takeoff_waypoints(self.target_alt)           
            self.srv_wp_push(waypoints)
            self.launched = True

        if abs(float(self.drone.altitude) - self.target_alt) < 0.5:
            self.time_mode_started = millis()
            self.srv_mode(0, '5')
            self.drone.mode = 3

    ###
    # Mode 3: waits for user to select object to track
    ###
    def track(self):
        self.drone.x = 1500 - self.axes[0] * 300
        self.drone.y = 1500 - self.axes[1] * 300
        self.drone.z = 1500

        if millis() - self.time_mode_started > 10000:
            (self.drone.x, self.drone.y, self.drone.z) = (1500, 1500, 1500)
            self.time_mode_started = millis()
            self.srv_mode(0, '3')
            self.drone.mode = 4

    ###
    # Mode 4: return to launch
    ###
    def rtl(self):
        if not self.returning:
            (lat, lon) = (self.platform_gps[0], self.platform_gps[1])
            platform_wp1 = mission_parser.make_waypoint(3, 16, True , lat, lon, self.target_alt, 10)
            platform_wp2 = mission_parser.make_waypoint(3, 16, False, lat, lon, self.target_alt, 10)
            waypoints = [platform_wp1, platform_wp2]
            self.srv_wp_push(waypoints)
            self.time_mode_started = millis()
            self.returning = True
        else:
            platform_pt = gps_tools.Point(self.platform_gps[1], self.platform_gps[0])
            drone_pt    = gps_tools.Point(self.drone.longitude, self.drone.latitude)
            distance = gps_tools.distance(platform_pt, drone_pt)

            if distance < 0.003 and millis() - self.time_mode_started > 10000:
                self.time_mode_started = millis()
                self.srv_mode(0, '5')
                self.drone.mode = 5

    def land(self):
    #        if self.fiducial == (0.0, 0.0, 0.0) and  millis() - self.time_mode_started > 3000: # Tries to find the landing platform again
    #            self.srv_mode(0, '3')
    #            self.time_mode_started = millis()
    #            self.returning = False
    #            self.drone.mode = 4

        # Moves faster in the xy plane, if drone is higher in the air
        self.srv_mode(0, '5')
        if self.fiducial[2] > 10000:
            scale = 100
        else:
            scale = 300

        self.t0 = self.t1
        self.t1 = millis()
        dt = (self.t1 - self.t0) / 1000.

        error_x = self.fiducial[0]
        error_y = self.fiducial[1]
        self.error_x_sum += error_x * dt
        self.error_y_sum += error_y * dt
        if self.error_x_prev == 0.0 and self.error_y_prev == 0.0:
            self.error_x_prev = error_x
            self.error_y_prev = error_y

        #                                        PID vals
        P_x = error_x                            * 0.80
        P_y = error_y                            * 0.50
        I_x = self.error_x_sum                   * 0.
        I_y = self.error_y_sum                   * 0.
        D_x = (error_x - self.error_x_prev) / dt * 0.1
        D_y = (error_y - self.error_y_prev) / dt * 0.06

        self.error_x_prev = error_x
        self.error_y_prev = error_y

        PID_x = P_x + I_x + D_x
        PID_y = P_y + I_y + D_y

        self.drone.x = int(round(1500 + PID_x * scale))
        self.drone.y = int(round(1500 + PID_y * scale))
        self.drone.z = 1500 

        # Prevents publishing outside of RC range
        if self.drone.x > 1800:
            self.drone.x = 1800
        elif self.drone.x < 1200:
            self.drone.x = 1200
        if self.drone.y > 1800:
            self.drone.y = 1800
        elif self.drone.y < 1200:
            self.drone.y = 1200

        if abs(self.drone.x - 1500) < 20 and abs(self.drone.y - 1500) < 20:
            self.drone.z = 1300
            if millis() - self.time_mode_started > 40000:
                self.time_mode_started = millis()
                self.drone.mode = 6
            elif millis() - self.time_mode_started > 30000:
                self.drone.z = 1000
        else:
            self.time_mode_started = millis()

        print (self.drone.x, self.drone.y, self.drone.z, self.fiducial)

    ###
    # Mode 6: disarm
    ###
    def disarm(self):
        self.reset_variables()
        self.failsafe = True
        self.srv_arm(False)
        self.drone.mode = 0

    ###
    # Publishes RC commands to the vehicle, depending on what mode it is currently in
    ###
    def fly(self): 
        if self.buttons:
            # Button 1 - enters failsafe mode (enables full control)
            if self.buttons[ ctrl['failsafe'] ]:
                self.reset_variables()
                self.failsafe = True
                print "Failsafe ON"

            # Button 3 - arms the drone
            if self.buttons[ ctrl['arm'] ]:
                self.srv_arm(True)
                print "Arming drone"

            # Button 4 - disarms drone
            if self.buttons[ ctrl['disarm'] ]:
                self.srv_arm(False) 
                print "Disarming drone"

            # Button 5 - initiate autonomy routine
            if self.buttons[ ctrl['auto'] ]:
                if self.drone.armed:
                    self.drone.mode = 2
                else:
                    self.drone.mode = 1 
                self.failsafe = False
                self.time_mode_started = millis()
                print "Beginning finite state machine"
                
            # Button 6 - end autonomy routine (RTL)
            if self.buttons[ ctrl['manual'] ]:
                self.drone.mode = 5
                self.failsafe = False
                self.time_mode_started = millis()
                self.t1 = millis()
                print "Returning to launch and landing"

            # Debounces button, once pressed
            time.sleep(0.1)

        # Initiates finite state machines
        if not self.failsafe:
            # Arming
            if self.drone.mode == 1:
                if self.outdoors:
                    self.srv_mode(0, '5')
                else:
                    self.srv_mode(0, '2')
                self.arm()

            # Launching
            if self.drone.mode == 2:
                self.launch()

            # Tracking
            if self.drone.mode == 3:
                self.track()

            # Returning to launch
            if self.drone.mode == 4:
                self.rtl()

            # Landing
            if self.drone.mode == 5:
                self.land()

            # Disarming
            if self.drone.mode == 6:
                self.disarm()

        else:
            # Reads joystick values
            if self.outdoors:
                self.srv_mode(0, '5')
            else:
                self.srv_mode(0, '2')
            self.drone.mode = 0

            x = 1500 - self.axes[ ctrl['x'] ] * 300
            y = 1500 - self.axes[ ctrl['y'] ] * 300
            z = 1500 + self.axes[ ctrl['z'] ] * 500
            yaw = 1500 - self.axes[ ctrl['yaw'] ] * 300

            if self.just_armed:
                z = 1000
                if abs(self.axes[ ctrl['z'] ]) > 0.1:
                    self.just_armed = False

            if z < 1200:
                z = 1000
            elif z < 1450:
                z = 1300
            elif z > 1650:
                z = 1750
            else:
                z = 1500

            if abs(x - 1500) < 50:
                x = 1500
            if abs(y - 1500) < 50:
                y = 1500

            (self.drone.x, self.drone.y, self.drone.z, self.drone.yaw) = (x, y, z, yaw)

        # Publishes commands
        if self.drone.armed:
            rc_msg = OverrideRCIn()
            rc_msg.channels = [self.drone.x, self.drone.y, self.drone.z, self.drone.yaw, 1400, self.drone.cam_tilt, self.drone.cam_tilt, self.drone.cam_tilt] 
            self.pub_rc.publish(rc_msg) 

        self.pub_mode.publish(self.drone.mode)

###
# Calculates the number of milliseconds since the epoch
###
def millis():
    return int(round(time.time() * 1000))

if __name__ == '__main__':
    try:
        var = Snotbot()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
