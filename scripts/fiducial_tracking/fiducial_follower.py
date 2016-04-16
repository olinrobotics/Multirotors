import rospy
import tf
from ar_pose.msg import ARMarkers
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point

from drone import *

from dynamic_reconfigure.server import Server #for PID tuning
from multirotors.cfg import PIDConfig

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

        self.P, self.I, self.D = 0.05, 0.005, 0.0005

        self.set_params_x(self.P, self.I, self.D)
        self.set_params_y(self.P, self.I, self.D)
        self.x_dir = -1
        self.y_dir = 1
        srv = Server(PIDConfig, self.PID_callback)

    def PID_callback(self, config, level):
        self.P = config.P
        self.I = config.I
        self.D = config.D
        self.set_params_x(self.P, self.I, self.D)
        self.set_params_y(self.P, self.I, self.D)
        return config

    """ Publishes image to image_raw """
    def run(self):
        self.pub_state_x.publish(Float64(self.fiducial_position.x))
        self.pub_state_y.publish(Float64(self.fiducial_position.y))

        if self.drone.mode == 'fiducial' and self.fiducial_id != -1:
            self.pub_pid_enable.publish(Bool(True))
            self.fly()
        else:
            self.pub_pid_enable.publish(Bool(False))
            self.drone.fly_joystick()

    """ Executes the control algorithms """
    def fly(self):
        x, y, z, yaw = 1500, 1500, 1500, 1500
        x += self.control_x * 300 * self.x_dir
        y += self.control_y * 300 * self.y_dir

        print x,y
        commands = [x, y, z, yaw, 0, 0, 0, 0]
        self.drone.publish_rc(commands)

    """ Sets PID parameters """
    def set_params_x(self, P, I, D):
        rospy.set_param('/landing_pid_x/Kp', P)
        rospy.set_param('/landing_pid_x/Ki', I)
        rospy.set_param('/landing_pid_x/Kd', D)

    def set_params_y(self, P, I, D):
        rospy.set_param('/landing_pid_y/Kp', P)
        rospy.set_param('/landing_pid_y/Ki', I)
        rospy.set_param('/landing_pid_y/Kd', D)

    ''' Sets new parameters if fiducial changes '''
    def set_new_params(self, fiducial):
        if fiducial.id == 0:
            self.set_params_x(self.P, self.I, self.D)
            self.set_params_y(self.P, self.I, self.D)
        elif fiducial.id == 1:
            self.set_params_x(self.P, self.I, self.D)
            self.set_params_y(self.P, self.I, self.D)
        elif fiducial.id == 2:
            self.set_params_x(self.P, self.I, self.D)
            self.set_params_y(self.P, self.I, self.D)

    """ Checks if drone has landed """
    def finished(self):
        return False

    """ Callback function for fiducial (grabs the most nested fiducial - highest id) """
    def fiducial_callback(self, data):
        fiducials = data.markers

        highest_conf = 0
        curr_fiducial = None
        for i in fiducials:
            if i.confidence > highest_conf:
                highest_conf = i.confidence
                curr_fiducial = i

        # Gets the position / orientation of the fiducial (if one is found)
        if curr_fiducial:
            # Changes PID values if fiducial changes
            if self.fiducial_id != curr_fiducial.id:
                self.set_new_params(curr_fiducial)

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