#!/usr/bin/env python

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import roslib
import rospy

from geometry_msgs.msg import Twist
from pan_tilt_zoom.msg import pan_tilt_goal

import sys, select, termios, tty


class teleop_seekur:

    def __init__(self):

        self.msg = """---------------------------------------------------
        Seekur Jr teleop
        Reading from the keyboard and Publishing to Twist!
        
        Movement keys:
        
        Moving arrows: <  ^  >
        1/z : increase/decrease max speeds by 10%
        2/x : increase/decrease only linear speed by 10%
        3/c : increase/decrease only angular speed by 10%
        
        PTZ keys:
        
        Moving PTZ (pan/tilt): w, a, s, d
        + zoom : e 
        -zoom : q
        
        PTZ step size:
        t/b : increase/decrease zoom step
        y/n : increase/decrease pan/tilt step
        
        
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

        self.ptBindings = {'w': (1, 0, 0, 0, 0),
                           'a': (0, -1, 0, 0, 0, 0),
                           's': (-1, 0, 0, 0, 0, 0),
                           'd': (0, 1, 0, 0, 0, 0),
                           'e': (0, 0, 1, 0, 0),
                           'q': (0, 0, -1, 0, 0),
                           't': (0, 0, 0, 0, 100),
                           'b': (0, 0, 0, 0, -100),
                           'y': (0, 0, 0, 3, 0),
                           'n': (0, 0, 0, -3, 0)}


        # Movement variables

        self.moveBindings = {'\x1b[A': (1, 0, 0, 0),
                             '\x1b[D': (0, 0, 0, 1),
                             '\x1b[C': (0, 0, 0, -1),
                             '\x1b[B': (-1, 0, 0, 0)}

        self.speedBindings = {'1': (1.1, 1.1),
                              'z': (.9, .9),
                              '2': (1.1, 1),
                              'x': (.9, 1),
                              '3': (1, 1.1),
                              'c': (1, .9)}


        # Initializing ROS publishers

        rospy.init_node('seekur_teleop_node')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.ptz_pub = rospy.Publisher('pan_tilt_zoom/pan_tilt_goal', pan_tilt_goal, queue_size=1)

        self.settings = termios.tcgetattr(sys.stdin)

        self.speed = 1.0
        self.turn = 1.0

        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0

        self.init_node()

    def init_node(self):

        ptz = pan_tilt_goal()
        twist = Twist()
        self.ptz_pub.publish(ptz)

        try:
            print(self.msg)

            while not rospy.is_shutdown():
                key = self.getch(0.5)

                # Movement

                if key in self.moveBindings.keys():
                    self.x = self.moveBindings[key][0]
                    self.y = self.moveBindings[key][1]
                    self.z = self.moveBindings[key][2]
                    self.th = self.moveBindings[key][3]

                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]

                    print(self.vels(self.speed, self.turn))
                    if self.status == 14:
                        print(self.msg)
                    self.status = (self.status + 1) % 15

                # PTZ

                elif key in self.ptBindings.keys():
                    self.tilt = self.tilt + self.ptBindings[key][0] * self.step_pt
                    self.pan = self.pan + self.ptBindings[key][1] * self.step_pt
                    self.zoom = self.zoom + self.ptBindings[key][2] * self.step_zoom
                    self.step_pt = self.step_pt + self.ptBindings[key][3]
                    self.step_zoom = self.step_zoom + self.ptBindings[key][4]

                    if self.step_pt <= 0:
                        self.step_pt = 1

                    if self.step_zoom <= 0:
                        self.step_zoom = 10

                    # PTZ message

                    if self.pan > 180:
                        self.pan = 180
                    elif self.pan < -180:
                        self.pan = -180

                    if self.tilt > 60:
                        self.tilt = 60
                    elif self.tilt < -30:
                        self.tilt = -30

                    if self.zoom < 0:
                        self.zoom = 0
                    elif self.zoom > 32767:
                        self.zoom = 32767

                    ptz.pan = self.pan
                    ptz.tilt = self.tilt
                    ptz.zoom = self.zoom
                    self.ptz_pub.publish(ptz)

                else:
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.th = 0
                    if (key == '\x03'):
                        break

                # cmd_vel message

                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.th * self.turn
                self.cmd_vel_pub.publish(twist)



        except Exception as e:
            print(e)

        finally:

            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)



    def setup_term(self, fd, when=termios.TCSAFLUSH):
        mode = termios.tcgetattr(fd)
        mode[tty.LFLAG] = mode[tty.LFLAG] & ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(fd, when, mode)

    def getch(self, timeout=None):
        settings = termios.tcgetattr(sys.stdin)
        try:
            self.setup_term(sys.stdin)
            try:
                rw, wl, xl = select.select([sys.stdin], [], [], timeout)
            except select.error:
                return
            if rw:
                key = sys.stdin.read(1)

                if key == '\x1b':
                    key += sys.stdin.read(2)

                return key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    seekur = teleop_seekur()

