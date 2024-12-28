#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dubins.py:

"""
import numpy as np
import math
from tools.randomPoints import *


# Class Vehicle
class Dubins:
    """
    dubins()                                           Propeller step inputs
    dubins('headingAutopilot',psi_d,V_c,beta_c,tau_X)  Heading autopilot
    
    Inputs:
        psi_d: desired heading angle (deg)
        V_c: current speed (m/s)
        beta_c: current direction (deg)
        tau_X: surge force, pilot input (N)        
    """

    def __init__(
            self,
            controlSystem="stepInput",
            r=0,
            V_current=0,
            R=0.4,
            B=1,
            serial_number=0,
            shift=None,
            color='b',
            starting_point=None
    ):
        # Initialize Dubins machine
        self.n_max = 10
        self.n_min = -5
        self.V_c = V_current
        self.R = R # wheel radius
        self.B = B # the distance between the wheels
        self.color = color
        self.L = 2.0  # Length (m)
        #self.B = 1.08  # beam (m)
        self.nu = np.array([0, 0, 0, 0, 0, 0], float)  # velocity vector
        self.u_actual = np.array([0, 0], float)  # propeller revolution states
        self.name = "Dubins Vehicle (see 'dubins.py' for more details)"
        self.controlDescription = "sigma"
        controlSystem = "Berman Law"
        self.serial_number = serial_number
        if shift is None:
            shift = np.array([0, 0], float)
        if starting_point is None:
            self.starting_point = np.array([0, 0], float) + np.array(shift, float)
        else:
            self.starting_point = np.array(starting_point, float) + np.array(shift, float)
        self.color = color

        self.controls = [
            "Left propeller shaft speed (rad/s)",
            "Right propeller shaft speed (rad/s)"
        ]
        self.dimU = len(self.controls)

        # Vehicle parameters
        m = 55.0  # mass (kg)
        pass


    def __str__(self):
        return ("░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░\n")

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        dx = np.cos(nu[3]) * (u_control[0] + u_control[1]) / 2 * self.R
        dy = np.sin(nu[3]) * (u_control[0] + u_control[1]) / 2 * self.R
        dtheta = (u_control[0] - u_control[1]) * self.R / self.B
        return np.array([dy, dx, 0, dtheta, 0, 0]), u_actual

    def controlAllocation(self, tau_X, tau_N):
        """
        [n1, n2] = controlAllocation(tau_X, tau_N)
        """
        tau = np.array([tau_X, tau_N])  # tau = B * u_alloc
        u_alloc = np.matmul(self.Binv, tau)  # u_alloc = inv(B) * tau

        # u_alloc = abs(n) * n --> n = sign(u_alloc) * sqrt(u_alloc)
        n1 = np.sign(u_alloc[0]) * math.sqrt(abs(u_alloc[0]))
        n2 = np.sign(u_alloc[1]) * math.sqrt(abs(u_alloc[1]))

        return n1, n2

    def repositioning(self, eta, nu, sample_time):
        # print(f'eta = {eta}')
        # print(f'nu = {nu}')
        return eta+nu*sample_time