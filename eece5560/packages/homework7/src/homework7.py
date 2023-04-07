#!/usr/bin/env python3
#
# file: homework7.py
# package: homework7
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#

import rospy
import math
from odometry_hw.msg import DistWheel, Pose2D

WHEEL_BASELINE = 0.1


class PoseCalculator:

    def __init__(self):

        self._subscriber = rospy.Subscriber(
            '/dist_wheel', DistWheel, self.callback
        )
        self._pub = rospy.Publisher(
            '/pose', Pose2D, queue_size=10
        )
        self._x_pos = 0
        self._y_pos = 0
        self._theta = 0
        return

    def callback(self, wheel_distances: DistWheel):

        sl = wheel_distances.dist_wheel_left
        sr = wheel_distances.dist_wheel_right

        delta_s = (sl + sr) / 2
        delta_theta = (sr - sl) / WHEEL_BASELINE
        delta_x = delta_s * math.cos(self._theta + delta_theta / 2)
        delta_y = delta_s * math.sin(self._theta + delta_theta / 2)

        self._x_pos = self._x_pos + delta_x
        self._y_pos = self._y_pos + delta_y
        self._theta = self._theta + delta_theta

        pose_msg = Pose2D()
        pose_msg.x = self._x_pos
        pose_msg.y = self._y_pos
        pose_msg.theta = self._theta
        self._pub.publish(pose_msg)

        return


if __name__ == '__main__':

    rospy.init_node('homework7_node', anonymous=True)
    pc = PoseCalculator()
    rospy.spin()
