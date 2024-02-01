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
from vehicles import *
from lib import *

# Simulation parameters: 
sampleTime = 0.02                   # sample time [seconds]
N = 50000                           # number of samples

# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
numDataPoints = 500                  # number of 3D data points
FPS = 10                            # frames per second (animated GIF)
filename = '3D_animation.gif'       # data file for animated GIF
browser = 'chrome'                  # browser for visualization of animated GIF

###############################################################################
# Vehicle constructors
###############################################################################
printSimInfo()

vehicle = otter('headingAutopilot',100.0,0,-30.0,200.0)
printVehicleinfo(vehicle, sampleTime, N)

###############################################################################
# Main simulation loop 
###############################################################################


def main():    

    [simTime, simData] = simulate(N, sampleTime, vehicle)
    
    # plotVehicleStates(simTime, simData, 1)
    # plotControls(simTime, simData, vehicle, 2)
    plot3D(simData, numDataPoints, FPS, filename, 3)   
    
    """ Ucomment the line below for 3D animation in the web browswer. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))
    
    plt.show()
    plt.close()

main()
