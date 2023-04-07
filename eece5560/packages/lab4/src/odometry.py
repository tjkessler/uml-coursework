#!/usr/bin/env python3
#
# file: odometry.py
# package: lab4
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#

import rospy
import math
from time import time
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String

VEL_TO_METERS = 0.432
DB_WIDTH = 0.127  # 0.127 meters, 5 inches


class Odometry:

    def __init__(self):

        self._x_pos = 0
        self._y_pos = 0
        self._theta = 0
        self._prev_time = time()

        self._subscriber = rospy.Subscriber(
            '~wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.callback
        )
        self._pub = rospy.Publisher(
            '/lab4_pose', String, queue_size=10
        )

        return

    def callback(self, wheel_command: WheelsCmdStamped):

        vel_left = wheel_command.vel_left * VEL_TO_METERS
        vel_right = wheel_command.vel_right * VEL_TO_METERS

        curr_time = time()
        delta_t = curr_time - self._prev_time
        self._prev_time = curr_time

        sl = vel_left * delta_t
        sr = vel_right * delta_t

        delta_s = (sl + sr) / 2
        delta_theta = (sr - sl) / DB_WIDTH
        delta_x = delta_s * math.cos(self._theta + delta_theta / 2)
        delta_y = delta_s * math.sin(self._theta + delta_theta / 2)

        self._x_pos = self._x_pos + delta_x
        self._y_pos = self._y_pos + delta_y
        self._theta = self._theta + delta_theta

        pose_msg = 'x_pos: {} - y_pos: {} - theta: {}'.format(
            self._x_pos, self._y_pos, math.degrees(self._theta) % 360
        )
        self._pub.publish(pose_msg)

        return
