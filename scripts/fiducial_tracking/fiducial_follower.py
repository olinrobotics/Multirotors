import rospy
import tf
from ar_pose.msg import ARMarkers
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point

from drone import *

CONF_THRESHOLD = 50

class FiducialFollower():
    def __init__(self, drone):
        self.drone = drone
        self.fiducial_id = -1
        self.fiducial_position = Point()
        self.fiducial_orientation = None
        self.control_x = 0.0
        self.control_y = 0.0

        # Subscribers
        rospy.Subscriber('/ar_pose_marker', ARMarkers, self.fiducial_callback, queue_size=10)
        rospy.Subscriber('/control_x', Float64, self.control_x_callback, queue_size=10)
        rospy.Subscriber('/control_y', Float64, self.control_y_callback, queue_size=10)

        # Publishers
        self.pub_state_x = rospy.Publisher('/state_x', Float64, queue_size=10)
        self.pub_state_y = rospy.Publisher('/state_y', Float64, queue_size=10)
        self.pub_pid_enable = rospy.Publisher('/pid_enable', Bool, queue_size=10)
        self.pub_setpoint = rospy.Publisher('/setpoint', Float64, queue_size=10)

        self.pub_setpoint.publish(Float64(0.0))

    """ Publishes image to image_raw """
    def run(self):
        self.pub_state_x.publish(Float64(self.fiducial_position.x))
        self.pub_state_y.publish(Float64(self.fiducial_position.y))

        if self.drone.axes[5] < -0.8 and self.fiducial_id != -1:
            self.pub_pid_enable.publish(Bool(True))
            self.fly()
        else:
            self.pub_pid_enable.publish(Bool(False))
            self.drone.fly_joystick()

    """ Executes the control algorithms """
    def fly(self):
        if self.fiducial_id == 0:
            self.set_params_x(0.5, 0.0, 0.0)
            self.set_params_y(0.5, 0.0, 0.0)
        elif self.fiducial_id == 1:
            self.set_params_x(0.5, 0.0, 0.0)
            self.set_params_y(0.5, 0.0, 0.0)
        elif self.fiducial_id == 2:
            self.set_params_x(0.5, 0.0, 0.0)
            self.set_params_y(0.5, 0.0, 0.0)
        else:
            self.drone.just_armed = False
            self.drone.fly_joystick(x=1500, y=1500, z=1500, yaw=1500)

    """ Sets PID parameters """
    def set_params_x(self, P, I, D):
        rospy.set_param('/landing_pid_x/Kp', P)
        rospy.set_param('/landing_pid_x/Ki', I)
        rospy.set_param('/landing_pid_x/Kd', D)

    def set_params_y(self, P, I, D):
        rospy.set_param('/landing_pid_y/Kp', P)
        rospy.set_param('/landing_pid_y/Ki', I)
        rospy.set_param('/landing_pid_y/Kd', D)

    """ Checks if drone has landed """
    def finished(self):
        return False

    """ Callback function for fiducial (grabs the most nested fiducial - highest id) """
    def fiducial_callback(self, data):
        fiducials = data.markers

        highest_id = -1
        curr_fiducial = None
        for i in fiducials:
            if i.id > highest_id:
                highest_id = i.id
                curr_fiducial = i

        # Gets the position / orientation of the fiducial (if one is found)
        if curr_fiducial:
            self.fiducial_id = curr_fiducial.id
            self.fiducial_position = curr_fiducial.pose.pose.position
            self.fiducial_orentation = curr_fiducial.pose.pose.orientation
        else:
            self.fiducial_id = -1

    """ Callback function for the contorl signals """
    def control_x_callback(self, data):
        self.control_x = data.data

    def control_y_callback(self, data):
        self.control_y = data.data