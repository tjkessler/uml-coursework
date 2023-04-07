#!/usr/bin/env python3
#
# file: lab4_line.py
# package: lab4
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#

import rospy
from duckietown_msgs.msg import Twist2DStamped
from odometry import Odometry


def drive_in_line():

    od = Odometry()
    r = rospy.Rate(10)
    pub = rospy.Publisher(
        'vehicle_command',
        Twist2DStamped,
        queue_size=1
    )
    movement = Twist2DStamped()
    for _ in range(20):
        r.sleep()
        movement.v = 0.47
        movement.omega = 0.0
        pub.publish(movement)
    movement.v = 0.0
    movement.omega = 0
    pub.publish(movement)


if __name__ == '__main__':

    rospy.init_node('lab4', anonymous=True)
    drive_in_line()
