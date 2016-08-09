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

#dictionary for stick controls (not implemented yet)
stick_controls = {'[A':'forward', '[B':'backward', '[C':'right', '[D':'left', #these are arrows
            'w': 'forward', 's': 'backward', 'd': 'right', 'a': 'left',
            'q':'up', 'z':'down'}

#dictionary for toggle controls (\x0D = enter)
toggle_controls = {' ':'rtl', '\x0D':'land', 'r':'arm', 't':'disarm', 'u':'takeoff',
            'm':'stabilize', 'l':'loiter', 'o':'auto', ',':'alt_hold', 'g':'guided', 'p':'planner', 'f':'fiducial'}
flight_modes = ['stabilize', 'alt_hold', 'loiter', 'auto', 'guided']

doubles = ['\x1B']

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_handler')
    
    """ ROS publishers """
    stick_pub = rospy.Publisher('stick_cmds', stick_cmd, queue_size=10)
    toggle_pub = rospy.Publisher('toggle_cmds', toggle_cmd, queue_size=10)
    #publish to control topics

    while not rospy.is_shutdown():
        """ get key """
        key = getKey()
        if key == '\x03': #break on ctrl+c
            break
        if key in doubles: #handles arrow keys because they are represented by two characters
            key = getKey(2)

        """ start of implementation of joystick style keyboard control """
        if key in stick_controls and no_joy:
            sticks = stick_cmd()
            sticks.roll = 1500
            sticks.pitch = 1500
            sticks.yaw = 1500
            sticks.throttle = 1500
            if stick_controls[key] == 'forward':
                sticks.pitch = 1300
            elif stick_controls[key] == 'backward':
                sticks.pitch = 1700
            #stick_pub.publish(sticks)

        """ keyboard control of toggle commands """
        elif key in toggle_controls:
            toggles = toggle_cmd() #initialize toggle command
            cmd = toggle_controls[key]
            #set appropriate flags
            if cmd == 'rtl':
                toggles.rtl = True
            elif cmd == 'land':
                toggles.land = True
            elif cmd == 'arm':
                toggles.arm = True
            elif cmd == 'disarm':
                toggles.disarm = True
            elif cmd in flight_modes:
                toggles.mode = cmd
            elif cmd == 'takeoff':
                toggles.takeoff = True
            elif cmd == 'planner':
                toggles.planner = True
            elif cmd == 'fiducial':
                toggles.fiducial = True
            toggle_pub.publish(toggles) #publish commands
            time.sleep(.2) #sleep for debounce
            toggle_pub.publish(toggle_cmd()) #publish a null command

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) #stop messing with terminal on exit


