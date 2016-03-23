#!/usr/bin/env python
import rospy
from drone import Drone
from fiducial_tracking.fiducial_follower import FiducialFollower
from mission_planning.mission_planner import *

from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from mavros_msgs.msg import RCIn, OverrideRCIn


class Arbiter():
    """ Arbiter controls which states are currently active """
    def __init__(self):
        self.rc_disable = False
        self.rc_overridden = True

        rospy.init_node('arbiter')

        self.sub_rc = rospy.Subscriber('/drone/rc/in', RCIn, self.rc_callback)

        self.pub_rc = rospy.Publisher('/drone/rc/override', OverrideRCIn, queue_size=10)

        self.robot = Drone()

        # Decides which actions need to run
        # state = FiducialFollower(robot)
        # state = Map_Planner(robot)

        # Loops until task has been fully completed

    def rc_callback(self, data):
        val = data.channels[5]
        self.rc_disable = val > 1500

    def go(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown() or not state.finished():
            if not self.rc_disable:
                # state.run()
                self.robot.fly_joystick()
                self.rc_overridden = True
            else:
                self.pub_rc.publish([0]*8)
                self.rc_overridden = False
            r.sleep()

if __name__ == '__main__':
    ready_set = Arbiter()
    ready_set.go()

