#!/usr/bin/env python3
#
# file: run.py
# package: homework5
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#

# Imports
from pid import PIDController
import time
import rospy
from std_msgs.msg import Float32


class HW5Node:

    def __init__(self):
        ''' HW5Node: subscribes to the /error topic, adjusts vehicle control
        based on PID controller calculation
        '''

        rospy.Subscriber('/error', Float32, self.callback)
        self._pub = rospy.Publisher('/control_input', Float32, queue_size=10)
        self._pid = PIDController()
        self._pid.set_gains(0.01, 0.03, 0.4)
        self._prev_time = time.time()
        rospy.set_param('/controller_ready', 'true')

    def callback(self, error: Float32):
        ''' HW5Node.callback: when /error is updated, calculate and publish
        vehicle control signal
        '''

        curr_time = time.time()
        time_delta = curr_time - self._prev_time
        self._prev_time = curr_time
        self._pub.publish(self._pid.control_signal(error, time_delta))


if __name__ == '__main__':

    rospy.init_node('homework5_node', anonymous=True)
    hw5n = HW5Node()
    rospy.spin()
