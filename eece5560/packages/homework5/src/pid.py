#!/usr/bin/env python3
#
# file: pid.py
# package: homework5
# author: Travis Kessler (Travis_Kessler@student.uml.edu)
#


class PIDController:

    def __init__(self):
        ''' PIDController: calculates control signal based on error, specified
        gain values
        '''

        self._ki = 0.5
        self._kp = 0.5
        self._kd = 0.5
        self._integral = 0
        self._prev_error = 0

    def set_gains(self, ki, kp, kd):
        ''' PIDController.set_gains: sets the gain values for the PID controller
        '''

        self._ki = ki
        self._kp = kp
        self._kd = kd

    def control_signal(self, error, delta_t):
        ''' PIDController.control_signal: calculates the control signal given
        supplied error and time differential
        '''

        error = error.data
        self._integral += error * delta_t
        deriv = (error - self._prev_error) / delta_t
        output = self._kp * error + self._ki * self._integral + self._kd * deriv
        self._prev_error = error
        return output
