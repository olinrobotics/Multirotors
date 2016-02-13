#!/usr/bin/env python
import rospy
from drone import Drone
from fiducial_tracking.fiducial_follower import FiducialFollower

""" Arbiter controls which states are currently active """
if __name__ == '__main__':
	rospy.init_node('arbiter')

	robot = Drone()

	# Decides which actions need to run
	state = FiducialFollower(robot)

	# Loops until task has been fully completed
	r = rospy.Rate(30)
	while not rospy.is_shutdown() or not state.finished():
		state.run()
		r.sleep()