#!/usr/bin/env python3
#
# file: homework4_part2.py
# package: homework4
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#
# Communicates with the Fibonacci service and action server to make a request
# of 3 and 15 returned values in the sequence
#

# Imports
from example_service.srv import *
import example_action_server.msg
import actionlib

import rospy

import sys
from time import gmtime, sleep, strftime, time


def calc_fib_sequence_service(n_elem: int) -> 'sequence':
    ''' calc_fib_sequence_service: uses the existing Fibonacci sequence service
    to request a given number of sequence elements

    Args:
        n_elem (int): number of elements to request in the sequence

    Returns:
        sequence: result datatype from Fibonacci.srv
    '''

    rospy.wait_for_service('calc_fibonacci')
    try:
        calc_fib = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
        return calc_fib(n_elem - 1)
    except rospy.ServiceException as e:
        rospy.logwarn('Service call failed: {}'.format(e))
        return None


def calc_fib_sequence_actionserver(n_elem: int) -> 'sequence':
    ''' calc_fib_sequence_actionserver: uses the existing Fibonacci action
    server to request a given number of sequence elements

    Args:
        n_elem (int): number of elements to request in the sequence

    Returns:
        sequence: result datatype from Fibonacci.action
    '''

    client = actionlib.SimpleActionClient(
        'fibonacci', example_action_server.msg.FibonacciAction
    )
    client.wait_for_server()
    t0 = time()
    rospy.loginfo('Sending goal...')
    goal = example_action_server.msg.FibonacciGoal(order=n_elem - 1)
    client.send_goal(goal)
    client.wait_for_result()
    t1 = time()
    rospy.loginfo('Goal received. Delta T: {}'.format(t1 - t0))
    return client.get_result()



def run_service():
    ''' runtime for calling service
    NOTE: script is currently configured for 3 sequence values returned
    '''

    N_ELEM = 3

    t0 = time()
    rospy.loginfo('Starting Fibonacci at {}'.format(
        strftime('%H:%M:%S.%f', gmtime())[:-3]
    ))
    result = calc_fib_sequence_service(N_ELEM)
    t1 = time()
    rospy.loginfo('Fibonacci ended at {}'.format(
        strftime('%H:%M:%S.%f', gmtime())[:-3]
    ))
    rospy.loginfo('Delta T: {}'.format(t1 - t0))
    rospy.loginfo('Returned value: {}'.format(result))


def run_actionserver():
    ''' runtime for calling action server
    NOTE: script is currently configured for 3 sequence values returned
    '''

    N_ELEM = 3

    t0 = time()
    rospy.loginfo('Starting Fibonacci at {}'.format(
        strftime('%H:%M:%S.%f', gmtime())[:-3]
    ))
    result = calc_fib_sequence_actionserver(N_ELEM)
    t1 = time()
    rospy.loginfo('Fibonacci ended at {}'.format(
        strftime('%H:%M:%S.%f', gmtime())[:-3]
    ))
    rospy.loginfo('Delta T: {}'.format(t1 - t0))
    rospy.loginfo('Returned value: {}'.format(result))



if __name__ == '__main__':

    rospy.init_node('homework4_part2', anonymous=True)
    rospy.loginfo('----- REQUESTING FROM SERVICE -----')
    sleep(0.5)
    run_service()
    sleep(0.5)
    rospy.loginfo('-- REQUESTING FROM ACTION SERVER --')
    sleep(0.5)
    run_actionserver()
