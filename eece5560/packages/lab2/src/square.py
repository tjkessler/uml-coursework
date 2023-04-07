#!/usr/bin/env python3
#
# file: square.py
# package: lab2
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#
# Commands duckiebot to move in a 1m side-length square
#

import rospy
from duckietown_msgs.msg import Twist2DStamped


def _forward(pub, r):

    movement = Twist2DStamped()
    for _ in range(20):
        r.sleep()
        movement.v = 0.4
        movement.omega = 0.0
        pub.publish(movement)
    movement.v = 0.0
    movement.omega = 0
    pub.publish(movement)


def _turn_right(pub, r):

    movement = Twist2DStamped()
    for _ in range(5):
        r.sleep()
        movement.v = 0.0
        movement.omega = -7
        pub.publish(movement)
    movement.v = 0.0
    movement.omega = 0
    pub.publish(movement)


def drive_in_square():

    r = rospy.Rate(10)
    pub = rospy.Publisher(
        'vehicle_command',
        Twist2DStamped,
        queue_size=1
    )
    _forward(pub, r)
    _turn_right(pub, r)
    _forward(pub, r)
    _turn_right(pub, r)
    _forward(pub, r)
    _turn_right(pub, r)
    _forward(pub, r)
    _turn_right(pub, r)


if __name__ == '__main__':

    rospy.init_node('lab2_square', anonymous=True)
    drive_in_square()
