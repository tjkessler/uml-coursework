#!/usr/bin/env python3
#
# file: subscriber.py
# package: homework2
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#
# This module subscribes to the homework1 package, logging the sum aggregated
# by it
#

# Imports
import rospy
from std_msgs.msg import Float32


class HW2Subscriber:

    def __init__(self):
        ''' HW2Subscriber: subscribes to changes in the 'total' topic,
        aggregated from the homework1 package

        Args:
            None

        Returns:
            None
        '''

        rospy.Subscriber('/homework1/total', Float32, self.log_total)
        return

    def log_total(self, data: Float32):
        ''' log_total: whenever the 'total' topic is updated by the homework1
        package, the new value is logged

        Args:
            data (Float32): new value of 'total' raised by the homework1
                package

        Returns:
            None
        '''

        rospy.loginfo(rospy.get_caller_id() + 'New total: {}'.format(
            data.data
        ))
        return


if __name__ == '__main__':

    rospy.init_node('subscriber', anonymous=True)
    HW2Subscriber()
    rospy.spin()
