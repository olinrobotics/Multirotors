import rospy
from drone import Drone
import fiducial_tracking.fiducial_follower

""" Arbiter controls which states are currently active """
if __name__ == '__main__':
	robot = Drone()

	# Decides which actions need to run
	state = FiducialFollower(robot)

	# Loops until task has been fully completed
	r = rospy.Rate(1)
	while not rospy.is_shutdown() or not state.finished():
		state.run()
		r.sleep()