#!/usr/bin/env python3
#
# file: lab3_lane_controller.py
# package: lab3
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#

from pid import PIDController

import time

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose

V_MAX = 0.2
V_MIN = 0.01
O_MAX = 2.5


class LaneController:

    def __init__(self, name):

        self._name = name
        self._controller_d = PIDController()
        self._controller_phi = PIDController()
        self._controller_d.set_gains(0.0, 4.5, 0.0)
        self._controller_phi.set_gains(0.0, 7.5, 0.0)

        self._prev_time = time.time()

        self._pub_car_cmd = rospy.Publisher(
            '~car_cmd', Twist2DStamped, queue_size=10
        )

        self._lane_pose = rospy.Subscriber(
            '~lane_pose', LanePose, self.handle_pose, queue_size=1
        )

    def handle_pose(self, pose_msg: LanePose):

        rospy.logwarn('Travis Kessler\'s Lab3 lane following code')

        new_time = time.time()
        delta_t = new_time - self._prev_time
        self._prev_time = new_time

        err_d = pose_msg.d
        err_phi = pose_msg.phi

        control_sig_v = abs(self._controller_d.control_signal(err_d, delta_t))
        control_sig_o = self._controller_phi.control_signal(err_phi, delta_t)

        if control_sig_v > V_MAX:
            control_sig_v = V_MAX
        if control_sig_v < V_MIN:
            control_sig_v = V_MIN

        if control_sig_o > O_MAX:
            control_sig_o = O_MAX
        elif control_sig_o < -O_MAX:
            control_sig_o = -O_MAX

        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header
        car_control_msg.v = control_sig_v
        car_control_msg.omega = -control_sig_o

        self._pub_car_cmd.publish(car_control_msg)


if __name__ == '__main__':

    name = 'lab3_lane_controller'
    rospy.init_node(name, anonymous=True)
    lane_controller = LaneController(name)
    rospy.spin()
