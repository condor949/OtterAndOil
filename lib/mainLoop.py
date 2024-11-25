#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#TODO: delete file
"""
Main simulation loop called by main.py.

Author:     Thor I. Fossen
"""

import numpy as np
from .gnc import attitudeEuler
from tqdm import tqdm

###############################################################################
# Function printSimInfo(vehicle)
###############################################################################
def printSimInfo():
    """
    Constructors used to define the vehicle objects as (see main.py for details):
        DSRV('depthAutopilot',z_d)                                       
        frigate('headingAutopilot',U,psi_d)
        otter('headingAutopilot',psi_d,V_c,beta_c,tau_X)                  
        ROVzefakkel('headingAutopilot',U,psi_d)                          
        semisub('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)                       
        shipClarke83('headingAutopilot',psi_d,L,B,T,Cb,V_c,beta_c,tau_X)  
        supply('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)      
        tanker('headingAutopilot',psi_d,V_c,beta_c,depth)    
        remus100('depthHeadingAutopilot',z_d,psi_d,V_c,beta_c)
    """

    print('---------------------------------------------------------------------------------------')
    print('The Python Vehicle Simulator')
    print('---------------------------------------------------------------------------------------')
    print('1 - Deep submergence rescue vehicle (DSRV): controlled by a stern plane, L = 5.0 m')
    print('2 - Frigate: rudder-controlled ship described by a nonlinear Nomoto model, L = 100.0 m')
    print('3 - Otter unmanned surface vehicle (USV): controlled by two propellers, L = 2.0 m')
    print('4 - ROV Zefakkel: rudder-controlled ship described by a nonlinear Nomoto model, L = 54.0 m')
    print('5 - Semisubmersible: controlled by tunnel thrusters and main propellers, L = 84.5 m')
    print('6 - Ship: linear maneuvering model specified by L, B and T using the Clarke (1983) formulas')
    print('7 - Offshore supply vessel: controlled by tunnel thrusters and main propellers, L = 76.2 m')
    print('8 - Tanker: rudder-controlled ship model including shallow water effects, L = 304.8 m')
    print('9 - Remus 100: AUV controlled by stern planes, a tail rudder and a propeller, L = 1.6 m')
    print('---------------------------------------------------------------------------------------')





###############################################################################
# Function simulate(N, sampleTime, vehicle)
###############################################################################

f0 = 0
mu = 1


def oil_intensity(x, y):
    return -(x - 10) ** 2 - (y - 10) ** 2 + 30


def Ivan_law(f_current, f_prev, sampleTime):
    sigma = -np.sign((f_current - f_prev) / sampleTime + mu * np.tanh(f_current - f0))
    return sigma


def simulate(y,x,N, sampleTime, vehicle):
    DOF = 6  # degrees of freedom
    t = 0  # initial simulation time

    # Initial state vectors
    eta = np.array([y, x, 0, 0, 0, 0], float)  # position/attitude, user editable
    nu = vehicle.nu  # velocity, defined by vehicle class
    u_actual = vehicle.u_actual  # actual inputs, defined by vehicle class

    # Initialization of table used to store the simulation data
    simData = np.empty([0, 2 * DOF + 2 * vehicle.dimU], float)

    f_current = oil_intensity(0, 0)

    # Simulator for-loop
    for i in tqdm(range(0, N + 1), desc="Vehicle Simulation"):

        t = i * sampleTime  # simulation time

        f_prev = f_current,
        f_current = oil_intensity(eta[0], eta[1])
        sigma = Ivan_law(f_current, f_prev, sampleTime)

        # print(f_current)
        if sigma < 0:
            u_control = [vehicle.n_min, vehicle.n_max]
        elif sigma > 0:
            u_control = [vehicle.n_max, vehicle.n_min]
        else:
            u_control = [0, 0]

        signals = np.hstack((eta, nu, u_control, u_actual))
        simData = np.vstack([simData, signals])

        # Propagate vehicle and attitude dynamics
        [nu, u_actual] = vehicle.dynamics(eta, nu, u_actual, u_control, sampleTime)
        eta = attitudeEuler(eta, nu, sampleTime)

    # Store simulation time vector
    simTime = np.arange(start=0, stop=t + sampleTime, step=sampleTime)[:, None]

    return (simTime, simData)
