#!/usr/bin/env python3
#
# file: converter.py
# package: homework3
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#
# This module subscribes to the '/homework1/total' topic, converts the input
# number (assumend to be a measurement in feet) to either meters or smoots,
# controlled by the rosparameter '/conversion'. The resulting converted value
# is then published to the '/homework3/converted_val' topic, and is remapped
# to '/homework1/total' for homework 2's subscriber node.
#

# Imports
import rospy
from std_msgs.msg import Float32


class HW3Converter:

    def __init__(self):
        ''' HW3Converter: subscribes to changes in Homework1's 'total' topic,
        converts to a specified unit, and publishes the converted value to the
        '/homework3/converted_val' topic

        Args:
            None

        Returns:
            None
        '''

        rospy.Subscriber('/homework1/total', Float32, self.callback)
        self.pub = rospy.Publisher('/homework3/converted_val', Float32,
                                   queue_size=10)
        return

    @staticmethod
    def _feet_to_meters(feet: Float32) -> float:
        ''' staticmethod: _feet_to_meters: converts a measurement in feet to
        meters

        Args:
            feet (Float32): value to convert, in feet

        Returns:
            float: value converted from feet to meters
        '''

        return feet.data * 0.3048

    @staticmethod
    def _feet_to_smoots(feet: Float32) -> float:
        ''' staticmethod: _feet_to_smoots: converts a measurement in feet to
        smoots

        Args:
            feet (Float32): value to convert, in feet

        Returns:
            float: value converted from feet to smoots
        '''

        return feet.data * 0.179104

    def callback(self, data: Float32):
        ''' callback: upon receiving a total value from the '/homework1/total'
        topic, converts the value to a specified unit, publishes it to the
        '/homework3/converted_val' topic

        Args:
            data (Float32): updated value from '/homework1/total' topic

        Returns:
            None
        '''

        prm = None
        new_val = data
        if rospy.has_param('/conversion'):
            prm = rospy.get_param('/conversion')
            if prm == 'meters':
                new_val = self._feet_to_meters(data)
            elif prm == 'smoots':
                new_val = self._feet_to_smoots(data)
            elif prm == 'feet':
                pass
            else:
                rospy.logwarn('Unknown parameter value: {}'.format(prm))
                pass
        else:
            rospy.logwarn('No parameter `/conversion` found!')
        rospy.loginfo(
            rospy.get_caller_id()
            + ' Conversion Unit: {} | Input Value: {} | Output Value: {}'
            .format(prm, data, new_val)
        )
        return


if __name__ == '__main__':

    rospy.init_node('converter', anonymous=True)
    HW3Converter()
    rospy.spin()
