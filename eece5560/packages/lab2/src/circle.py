#!/usr/bin/env python3
#
# file: circle.py
# package: lab2
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#
# Commands duckiebot to move in a 1m diameter circle
#

import rospy
from duckietown_msgs.msg import Twist2DStamped


def drive_in_circle():

    r = rospy.Rate(10)
    pub = rospy.Publisher(
        'vehicle_command',
        Twist2DStamped,
        queue_size=1
    )
    movement = Twist2DStamped()
    for _ in range(80):
        r.sleep()
        movement.v = 0.4
        movement.omega = 2
        pub.publish(movement)
    movement.v = 0.0
    movement.omega = 0
    pub.publish(movement)


if __name__ == '__main__':

    rospy.init_node('lab2_circle', anonymous=True)
    drive_in_circle()
