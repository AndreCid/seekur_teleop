#!/usr/bin/env python

from sensor_msgs.msg import Joy
from pan_tilt_zoom.msg import pan_tilt_goal

import rospy



class teleop_seekur:

    def _init_(self):

        self.msg = """---------------------------------------------------
        Seekur Jr teleop - Joystick

        Movement keys:

        Allow movement : 'A'
        Movement : Left stick

        PTZ keys:

        Moving PTZ (pan/tilt): Right stick 

        + zoom : 'RB' 
        -zoom : 'LB'

        PTZ step size:

        Increase zoom step : 'RT'
        Decrease zoom step : 'LT'

        Increase pan/tilt step : 'Y'
        Decrease pan/tilt step : 'B'


        PTZ LIMITS:

        Pan : 180 to -180 degrees
        Tilt : -30 to 60 degrees
        Zoom : 0 to 32767


        CTRL-C to quit
        ---------------------------------------------------
        """

        # Pan, tilt, Zoom variables
        self.pan = 0.0
        self.tilt = 0.0
        self.zoom = 0.0

        self.step_pt = 5.0
        self.step_zoom = 200.0

        # Initializing ROS publishers

        rospy.init_node('seekur_teleop_node_joy')
        self.ptz_pub = rospy.Publisher('pan_tilt_zoom/pan_tilt_goal', pan_tilt_goal, queue_size=1)
        self.rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.r = rospy.Rate(5)
        self.init_node()

    def init_node(self):

        self.ptz = pan_tilt_goal()
        self.ptz_pub.publish(self.ptz)

        print(self.msg)

        while not rospy.is_shutdown():
            self.r.sleep()

    def joy_callback(self, data):

        # Seekur step zoom (+)
        if data.axes[2] != 0:
            self.step_zoom = self.step_zoom - 100
            if self.step_zoom <= 0:
                self.step_zoom = 10

        # Seekur step zoom (-)
        if data.axes[5] != 0:
            self.step_zoom = self.step_zoom + 100


        # Seekur step ptz (+)
        if data.buttons[3] == 1:
            self.step_pt = self.step_pt + 3


        # Seekur step ptz (-)
        if data.buttons[1] == 1:
            self.step_pt = self.step_pt - 3
            if self.step_pt <= 0:
                self.step_pt = 1

        # Pan controller
        if data.axes[3] != 0:
            self.pan = self.pan + (self.step_pt * (-data.axes[3]))
            if self.pan < -180:
                self.pan = -180
            elif self.pan > 180:
                self.pan = 180

        # Tilt controller
        if data.axes[4] != 0:
            self.tilt = self.tilt + (self.step_pt * data.axes[4])
            if self.tilt < -30:
                self.tilt = -30
            elif self.tilt > 60:
                self.tilt = 60

        # PTZ Zoom out
        if data.buttons[4] == 1:
            self.zoom = self.zoom - self.step_zoom
            if self.zoom < 0:
                self.zoom = 0

        # PTZ Zoom in
        if data.buttons[5] == 1:
            self.zoom = self.zoom + self.step_zoom
            if self.zoom > 32767:
                self.zoom = 32767

        self.ptz.pan = self.pan
        self.ptz.tilt = self.tilt
        self.ptz.zoom = self.zoom
        self.ptz_pub.publish(self.ptz)


if __name__ == "__main__":
    seekur = teleop_seekur()