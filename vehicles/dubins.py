#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dubins.py:

"""
import math
from .vehicle import *
from tools.random_generators import *


class Dubins(Vehicle):
    """
    dubins()                                           Propeller step inputs
    dubins('headingAutopilot',psi_d,V_c,beta_c,tau_X)  Heading autopilot
    
    Inputs:
        psi_d: desired heading angle (deg)
        V_c: current speed (m/s)
        beta_c: current direction (deg)
        tau_X: surge force, pilot input (N)        
    """
    name = 'dubins'
    def __init__(
            self,
            controlSystem="stepInput",
            r=0,
            V_current=0,
            R=0.315,
            B=1,
            serial_number=0,
            shift=None,
            color='b',
            starting_point=None
    ):
        super().__init__(V_current,
                         serial_number,
                         shift,
                         color,
                         starting_point)
        # Initialize Dubins machine
        self.n_max = 10
        self.n_min = -5
        self.R = R # wheel radius
        self.B = B # the distance between the wheels
        self.L = 2.0  # Length (m)
        self.nu = np.array([0, 0, 0, 0, 0, 0], float)  # velocity vector
        self.u_actual = np.array([0, 0], float)  # propeller revolution states
        self.type = "Dubins Vehicle (see 'dubins.py' for more details)"
        self.controlDescription = "sigma"

        self.controls = [
            "Left propeller shaft speed (rad/s)",
            "Right propeller shaft speed (rad/s)"
        ]
        self.dimU = len(self.controls)

    def __str__(self):
        return (f'---vehicle--------------------------------------------------------------------------\n'
                f'{self.type}\n'
                f'Length: {self.L} m\n'
                f'Control: {self.controlDescription}\n'
                f'Wheel radius: {self.R} m\n'
                f'Distance between the wheels: {self.B} m\n'
                f'Starting point: [{self.starting_point[0]}, {self.starting_point[1]}]')

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        dx = np.cos(eta[3]) * (u_control[0] + u_control[1]) / 2 * self.R
        dy = np.sin(eta[3]) * (u_control[0] + u_control[1]) / 2 * self.R
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