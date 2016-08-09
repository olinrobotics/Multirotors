#!/usr/bin/env python
""" get input from joystick and publish it to appropriate topics """

import rospy
from multirotors.msg import stick_cmd, toggle_cmd
from sensor_msgs.msg import Joy

# mappings for different joysticks we use, note: you can set arm and disarm to the same button
joystick = {'arm': 2, 'disarm': 3, 'failsafe': 0, 'auto': 4, 'manual': 5, 'x': 0, 'y': 1, 'z': 3, 'yaw': 2}
xbox     = {'arm': 0, 'disarm': 1, 'failsafe': 8, 'auto': 2, 'manual': 3, 'x': 3, 'y': 4, 'z': 1, 'yaw': 0}
rcsim    = {'arm': 2, 'disarm': 2, 'failsafe': 8, 'auto': 1, 'manual': 3, 'x': 0, 'y': 1, 'z': 2, 'yaw': 4}
rcsim_lim = [[-.707,.653],[-.587,.598],[-.620,.598],[-1,1],[-.772,.631],[-1,1]]

ctrl = joystick # Set this variable to the joystick you are currently using

class Joystick():
    """ handle inputs from a joystick and convert them to stick_cmd and toggle_cmd messages """
    def __init__(self):
        # Joystick Variables
        self.buttons = [0]*10 #initialize all 8 buttons as 0
        self.axes = [0]*8 #initialize all 8 axes as 0

        self.arm = False
        self.disarm = False
        self.last_arm = False #for debouncing
        self.last_disarm = False #for debouncing

        rospy.init_node('joy_handler')

        """ subscribers """
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback) #listen to joystick inputs

        """ publishers """
        self.pub_stick_cmd = rospy.Publisher('/stick_cmds', stick_cmd, queue_size=10)
        self.pub_toggle_cmd = rospy.Publisher('/toggle_cmds', toggle_cmd, queue_size=10)
        # publish to control topics

    def run(self):
        """ main run loop: handles inputs """
        # arm button handler
        arm_data = self.buttons[ctrl['arm']]
        self.arm, self.last_arm = self.debounce_button(arm_data, self.arm, self.last_arm)

        # disarm button handler
        disarm_data = self.buttons[ctrl['disarm']]
        self.disarm, self.last_disarm = self.debounce_button(disarm_data, self.disarm, self.last_disarm)

        roll = 1500 - self.axes[ ctrl['x'] ] * 300 #map from -1:1 to 1200:1800
        pitch = 1500 - self.axes[ ctrl['y'] ] * 300
        throttle = 1500 + self.axes[ ctrl['z'] ] * 500 #throttle gets full 1000:2000
        yaw = 1500 - self.axes[ ctrl['yaw'] ] * 200

        if throttle < 1100: #min throttle deadzone
            throttle = 1000
        elif abs(throttle - 1500) < 50: #throttle midpoint deadzone
            throttle = 1500
        #other deadzones
        if abs(yaw - 1500) < 50:
            yaw = 1500

        if abs(roll - 1500) < 50:
            roll = 1500
        if abs(pitch - 1500) < 50:
            pitch = 1500

        sticks = stick_cmd() #initialize stick command
        #set data
        sticks.roll = roll
        sticks.pitch = pitch
        sticks.yaw = yaw
        sticks.throttle = throttle
        self.pub_stick_cmd.publish(sticks) #publish stick command

        toggles = toggle_cmd() #initialize toggle command
        #set data
        toggles.arm = self.arm
        toggles.disarm = self.disarm
        self.pub_toggle_cmd.publish(toggles) #publish toggle command

        #reset booleans
        self.arm = False
        self.disarm = False

    def debounce_button(self, data, output, last):
        """ 
        Handle buttons with debounce - toggles on button press
        use as follows in loop which updates data:
        output, last = debounce_button(data, output, last)
        """
        if data:
            if not last:
                output = True
                last = True
            else:
                output = False
        else:
            output = False
            last = False
        return output, last

    def joy_callback(self, data):
        """ reset values on joystick input """
        self.axes = list(data.axes)
        self.buttons = data.buttons
        if ctrl == rcsim: #scale axis to -1:1 because they don't come in that way
            self.scale_axes(rcsim_lim)

    def scale_axes(self, limits):
        """ scale axes to -1:1 """
        for ch in range(len(self.axes)):
            if self.axes[ch] > 0:
                self.axes[ch] *= 1./limits[ch][1]
            else:
                self.axes[ch] *= -1./limits[ch][0]

if __name__ == '__main__':
    controller = Joystick()
    r = rospy.Rate(10) #run 10 times per second
    while not rospy.is_shutdown(): #run until ros stops
        controller.run()
        r.sleep()