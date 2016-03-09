#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('landing_setpoint')
    pub_setpoint = rospy.Publisher('/setpoint', Float64, queue_size=10)

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
    	pub_setpoint.publish(Float64(0.0))