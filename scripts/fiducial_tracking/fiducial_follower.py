import rospy
import tf
from ar_pose.msg import ARMarkers

from drone import *

CONF_THRESHOLD = 70

class FiducialFollower():
    def __init__(self, drone):
        self.drone = drone
        self.fiducial_id = -1
        self.fiducial_position = None
        self.fiducial_orientation = None

        rospy.Subscriber('/ar_pose_marker', ARMarkers, self.fiducial_callback, queue_size=10)

    """ Publishes image to image_raw """
    def run(self):
        if self.drone.buttons[0]:
            self.fly()
        else:
        	pass
            # self.drone.fly_joystick()

    """ Executes the control algorithms """
    def fly(self):
        if self.fiducial_id == 1:
            self.PID(1, 2, 3)
        elif self.fiducial_id == 9:
            self.PID(1, 2, 3)
        elif self.fiducial_id == 15:
            self.PID(1, 2, 3)
        else:
            self.drone.fly_joystick()

    """ Calculates control signals through PID """
    def PID(self, P, I, D):
    	x_error = -self.fiducial_position.x
    	y_error = -self.fiducial_position.y

    """ Checks if drone has landed """
    def finished(self):
        return False

    """ Callback function for fiducial (grabs the most nested fiducial - highest id) """
    def fiducial_callback(self, data):
        fiducials = data.markers

        highest_id = 0
        curr_fiducial = None
        for i in fiducials:
            if i.id > highest_id and i.confidence > CONF_THRESHOLD:
                highest_id = i.id
                curr_fiducial = i

        # Gets the position / orientation of the fiducial (if one is found)
        if curr_fiducial:
            self.fiducial_id = curr_fiducial.id
            self.fiducial_position = curr_fiducial.pose.pose.position
            self.fiducial_orentation = curr_fiducial.pose.pose.orientation
        else:
            self.fiducial_id = -1

if __name__ == '__main__':
    try:
        var = FiducialFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass