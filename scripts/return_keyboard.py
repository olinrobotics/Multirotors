#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from multirotors.msg import stick_cmd, toggle_cmd
import time

import sys, select, termios, tty

no_joy = False #set this to true when stick controls are implemented

def getKey(bytes=1):
    """ waits for a key press and returns the key as a string """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(bytes)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#dictionary for toggle controls (\x0D = enter)
toggle_controls = {' ':'fiducial','\x0D','loiter'}
flight_modes = ['stabilize', 'alt_hold', 'loiter', 'auto', 'guided']

doubles = ['\x1B']

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_handler')
    
    """ ROS publishers """
    toggle_pub = rospy.Publisher('toggle_cmds', toggle_cmd, queue_size=10)
    #publish to control topics

    while not rospy.is_shutdown():
        """ get key """
        key = getKey()
        if key == '\x03': #break on ctrl+c
            break
        if key in doubles: #handles arrow keys because they are represented by two characters
            key = getKey(2)


        """ keyboard control of toggle commands """
        if key in toggle_controls:
            if cmd == 'fiducial':
                toggles.fiducial = True
            elif cmd in flight_modes:
                toggles.mode = cmd
            toggle_pub.publish(toggles) #publish commands
            time.sleep(.2) #sleep for debounce
            toggle_pub.publish(toggle_cmd()) #publish a null command

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) #stop messing with terminal on exit


