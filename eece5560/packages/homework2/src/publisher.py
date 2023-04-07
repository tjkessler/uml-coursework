#!/usr/bin/env python3
#
# file: publisher.py
# package: homework2
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#
# This module publishes integers to the homework1 package
#

# Imports
import rospy
from std_msgs.msg import Float32


class HW2Publisher:

    def __init__(self):
        ''' HW2Publisher: publishes data, of type std_msgs.msg.Float32, to the
        Homework1 package

        Args:
            None

        Returns:
            None
        '''

        self.pub = rospy.Publisher('/homework1/delta', Float32, queue_size=10)
        return

    def send_float(self, data: Float32):
        ''' send_float: sends a std_msgs.msg.Float32 to Homework1 via the
        'delta' topic

        Args:
            data (Float32): data to be sent to Homework1

        Returns:
            None
        '''

        self.pub.publish(data)
        return


if __name__ == '__main__':

    try:
        pub = HW2Publisher()
        rospy.init_node('publisher', anonymous=True)
        rate = rospy.Rate(1)
        for i in range(1, 11):
            pub.send_float(i)
            rate.sleep()
        pass

    except rospy.ROSInterruptException:
        pass
