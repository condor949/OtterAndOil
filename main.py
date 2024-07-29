#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd edition, John Wiley & Sons, Chichester, UK. 
URL: https://www.fossen.biz/wiley  

Author:     Thor I. Fossen
"""
import os
import webbrowser
import matplotlib.pyplot as plt
import argparse
from vehicles import *
from lib import *

# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
numDataPoints = 500  # number of 3D data points
FPS = 10  # frames per second (animated GIF)
filename = '3D_animation.gif'  # data file for animated GIF
browser = 'chrome'  # browser for visualization of animated GIF

###############################################################################
# Main simulation loop 
###############################################################################


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='ProgramName',
        description='What the program does',
        epilog='Text at the bottom of help')

    sym_param = parser.add_argument_group('simulation parameters')
    sym_param.add_argument('-N', '--number-samples', metavar='N', default=20000, dest='N', help='number of samples')
    sym_param.add_argument('-sT', '--sample-time', metavar='sampleTime', default=0.02, dest='sampleTime',
                           help='sample time [seconds]')
    sym_param.add_argument('-C', '--cycles', metavar='cycles', default=1,
                           help='the number of repetitions of the experiment')
    veh_params = parser.add_argument_group('vehicle parameters')
    veh_params.add_argument('-V', '--v-current', metavar='V_current', default=0, dest='V_current',
                            help='current speed (m/s)')
    veh_params.add_argument('-bC', '--beta-current', metavar='beta_current', default=-30.0, dest='beta_current',
                            help='current direction (deg)')

    args = parser.parse_args()

    ###############################################################################
    # Vehicle constructors
    ###############################################################################
    # printSimInfo()

    vehicle = otter(V_current=args.V_current, beta_current=args.beta_current)
    printVehicleinfo(vehicle, args.sampleTime, args.N)

    [simTime, simData] = simulate(args.N, args.sampleTime, vehicle)

    # plotVehicleStates(simTime, simData, 1)
    # plotControls(simTime, simData, vehicle, 2)
    plot3D(simData, numDataPoints, FPS, filename, 3)

    """ Uncomment the line below for 3D animation in the web browser. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))

    plt.show()
    plt.close()
