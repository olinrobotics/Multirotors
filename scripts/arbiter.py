#!/usr/bin/env python
import rospy
from drone import Drone
#from fiducial_tracking.fiducial_follower import FiducialFollower
from fiducial_tracking.goto_target import GotoTarget
from mission_planning.mission_planner import *

from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from mavros_msgs.msg import RCIn, OverrideRCIn

class BaseFly():
    """ basic state of flying with a joystick """
    def __init__(self, drone):
        self.drone = drone #give the state the correct drone object
    def run(self):
        """ runs continuously in BaseFly state: runs joystick code """
        self.drone.fly_joystick()
    def finished(self):
        """ whether or not to move on to next state """
        return False


class Arbiter():
    """ Arbiter controls which states are currently active 
        and runs the current state's run method
        Also manages RC override of code
    """
    def __init__(self):
        self.rc_disable = False #true = RC controller took control back
        self.rc_overridden = True #tracks who has control currently

        rospy.init_node('arbiter')

        """ subscribers """
        self.sub_rc = rospy.Subscriber('/drone/rc/in', RCIn, self.rc_callback)
        #listens to RC controller inputs

        """ publishers """
        self.pub_rc = rospy.Publisher('/drone/rc/override', OverrideRCIn, queue_size=10)
        #overrides the RC inputs on the drone

        self.robot = Drone() #initialize drone object

        # Decides which actions need to run
          #currently there is no logic for switching states
          #just select the state you want to test from here
        #self.state = FiducialFollower(self.robot)
        # self.state = BaseFly(self.robot)
        self.state = GotoTarget(self.robot)

        # Loops until task has been fully completed

    def rc_callback(self, data):
        val = data.channels[5] #get value of channel 6 (index 5)
        self.rc_disable = val > 1500 #disable code if value is high

    def go(self):
        r = rospy.Rate(30) #run 30 times per second
        while not rospy.is_shutdown() and not self.state.finished():
        # run loop until a quit command or the state declares it is done
            if not self.rc_disable: #check if code is enabled
                self.state.run() #run the state's run method
                self.rc_overridden = True #track that RC commands are overridden
            else:
                if self.rc_overridden: #if RC controller took control back
                    self.pub_rc.publish([0]*8) #give all control back to RC
                self.rc_overridden = False #track that RC controller has control
            r.sleep()

if __name__ == '__main__':
    ready_set = Arbiter()
    ready_set.go()
