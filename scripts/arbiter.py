#!/usr/bin/env python
import rospy
from drone import Drone
from fiducial_tracking.fiducial_follower import FiducialFollower
from mission_planning.mission_planner import *

from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent


""" Arbiter controls which states are currently active """
if __name__ == '__main__':
    rospy.init_node('arbiter')

    robot = Drone()

    # Decides which actions need to run
    state = FiducialFollower(robot)
    # state = Map_Planner(robot)

    # Loops until task has been fully completed
    r = rospy.Rate(30)
    while not rospy.is_shutdown() or not state.finished():
        state.run()
        r.sleep()