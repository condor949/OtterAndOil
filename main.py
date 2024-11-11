#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Otter and Oil, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.
"""
import os
import json
import argparse
import webbrowser
from lib import *
from spaces import *
from vehicles import *
from controllers import *
from tools.dataStorage import *
from tools.randomPoints import *


# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
numDataPoints = 500  # number of 3D data points
FPS = 30  # frames per second (animated GIF)
filename = '3D_animation'  # data file for animated GIF
browser = 'chrome'  # browser for visualization of animated GIF

###############################################################################
# Main simulation loop 
###############################################################################


if __name__ == '__main__':

    # Define the expected types and set initial values for each variable
    clean_cache: bool = False
    big_picture: bool = False
    not_animated: bool = False
    space_filename: str = ""
    N: int = 0
    SampleTime: float = 0.0
    cycles: int = 0
    radius: int = 0
    catamarans: int = 0
    V_current: float = 0.0
    beta_current: float = 0.0


    # Load the JSON file and read data into corresponding variables
    def read_and_assign_parameters(input_filename):
        global clean_cache, big_picture, not_animated, space_filename, N
        global SampleTime, cycles, radius, catamarans, V_current, beta_current

        with open(input_filename, 'r') as file:
            data = json.load(file)

        # Assign values with appropriate types
        clean_cache = bool(data.get("clean_cache", clean_cache))
        big_picture = bool(data.get("big_picture", big_picture))
        not_animated = bool(data.get("not_animated", not_animated))
        space_filename = str(data.get("space_filename", space_filename))
        N = int(data.get("N", N))
        SampleTime = float(data.get("SampleTime", SampleTime))
        cycles = int(data.get("cycles", cycles))
        radius = int(data.get("radius", radius))
        catamarans = int(data.get("catamarans", catamarans))
        V_current = float(data.get("V_current", V_current))
        beta_current = float(data.get("beta_current", beta_current))


    # Save the variables to a new JSON file
    def save_parameters(output_filename):
        parameters = {
            "clean_cache": clean_cache,
            "big_picture": big_picture,
            "not_animated": not_animated,
            "space_filename": space_filename,
            "N": N,
            "SampleTime": SampleTime,
            "cycles": cycles,
            "radius": radius,
            "catamarans": catamarans,
            "V_current": V_current,
            "beta_current": beta_current
        }

        with open(output_filename, 'w') as file:
            json.dump(parameters, file, indent=4)


    # Example usage
    input_filename = 'input.json'
    output_filename = 'output.json'

    read_and_assign_parameters(input_filename)
    save_parameters(output_filename)

    parser = argparse.ArgumentParser(
        prog='Otter and Oil',
        description="The program performs a series of calculations of the catamaran's trajectory at the exit to the "
                    "target line",
        epilog='The data is stored by timestamps in the data/ directory')

    main_param = parser.add_argument_group('script parameters')
    main_param.add_argument('-cc', '--clean-cache', action='store_true', dest='clean_cache', default=False,
                            help='clean data directories')
    main_param.add_argument('-big', '--big-picture', action='store_true', dest='big_picture', default=False,
                            help='set parameters of animation x2')
    main_param.add_argument('-disanim', '--disable-animation', action='store_true', dest='not_animated', default=False,
                           help='enable picture only mode, disable animation')
    main_param.add_argument('-space', '--space-file', dest='space_filename', help='')

    sym_param = parser.add_argument_group('simulation parameters')
    sym_param.add_argument('-N', '--number-samples', metavar='N', default=20000, dest='N', type=int,
                           help='number of samples')
    sym_param.add_argument('-sT', '--sample-time', metavar='sampleTime', default=0.02, dest='sampleTime', type=float,
                           help='sample time [seconds]')
    sym_param.add_argument('-C', '--cycles', metavar='cycles', default=1, type=int,
                           help='the number of repetitions of the experiment')
    sym_param.add_argument('-R', '--start-radius', metavar='radius', default=1, type=int, dest='radius',
                           help='the radius of the catamaran starting position (m)')
    sym_param.add_argument('-NC', '--number-catamarans', metavar='catamarans', default=1, type=int, dest='catamarans',
                           help='the number of catamaran models participating in the experiment')

    veh_params = parser.add_argument_group('catamaran parameters')
    veh_params.add_argument('-V', '--v-current', metavar='V_current', default=0, dest='V_current', type=float,
                            help='current speed (m/s)')
    veh_params.add_argument('-bC', '--beta-current', metavar='beta_current', default=-30.0, dest='beta_current',
                            type=float, help='current direction (deg)')

    args = parser.parse_args()

    ###############################################################################
    # Vehicle constructors
    ###############################################################################
    # printSimInfo()

    vehicles = [otter(V_current=args.V_current, beta_current=args.beta_current) for _ in range(args.catamarans)]
    # printVehicleinfo(vehicle, args.sampleTime, args.N)

    # plotVehicleStates(simTime, simData, 1)
    # plotControls(simTime, simData, vehicle, 2)
    # [simTime, simData] = simulate(args.N, args.sampleTime, vehicle)
    # plot3D(simData, numDataPoints, FPS, filename, 3)

    """ Uncomment the line below for 3D animation in the web browser. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))
    if args.clean_cache:
        clean_data()
    if args.big_picture:
        print('BE CAREFUL THE BIG PICTURE MODE REQUIRES MORE MEMORY')
    start_points = [[0, 0]]
    space = Gaussian3DSpace(grid_size=numDataPoints,shift_x=11,shift_y=11,shift_z=-10)
    space.add_gaussian_peak(x0=0, y0=0, amplitude=30, sigma_x=6, sigma_y=6)
    space.add_gaussian_peak(x0=5, y0=5, amplitude=25, sigma_x=1.5, sigma_y=4)
    space.add_gaussian_peak(x0=-5, y0=-5, amplitude=20, sigma_x=3, sigma_y=2)
    space.add_gaussian_peak(x0=-3, y0=6, amplitude=28, sigma_x=1.5, sigma_y=1.5)
    space.add_gaussian_peak(x0=4, y0=-6, amplitude=12, sigma_x=1.8, sigma_y=1)
    space.drown()
    space.set_contour_points(plane_z=0, tol=1)
    # print(space.contour_points)
    for i in range(args.cycles):
        if args.catamarans > 1:
            start_points = generate_random_points(args.radius, args.catamarans)
        controller = IntensityBasedController(starting_points=start_points, sample_time=args.sampleTime, space=space)
        timestamped_suffix: str = create_timestamped_suffix()
        timestamped_folder: str = create_timestamped_folder(suffix=timestamped_suffix)
        # swarmData = []
        # for point in start_points:
        #     [simTime, simData] = simulate(point[0], point[1], args.N, args.sampleTime, vehicle)
        #     swarmData.append(simData)

        [simTime, swarmData] = simultaneous_simulate(vehicles=vehicles, initial_positions=start_points, N=args.N,
                                                     sampleTime=args.sampleTime, controller=controller)
        controller.create_intensity_graph(path=os.path.join(timestamped_folder,
                                                            create_timestamped_filename_ext("intensity",
                                                                                            timestamped_suffix,
                                                                                            "png")))
        controller.create_sigma_graph(path=os.path.join(timestamped_folder,
                                                            create_timestamped_filename_ext("sigmas",
                                                                                            timestamped_suffix,
                                                                                            "png")))
        controller.create_quality_graph(path=os.path.join(timestamped_folder,
                                                            create_timestamped_filename_ext("quality",
                                                                                            timestamped_suffix,
                                                                                            "png")),
                                        sample_time=args.sampleTime)
        plot3D(swarmData, numDataPoints, FPS, os.path.join(timestamped_folder,
                                                             create_timestamped_filename_ext(filename,
                                                                                             timestamped_suffix,
                                                                                             "gif")),
                 1, args.big_picture, space, args.not_animated)
        # # new_folder = create_timestamped_folder(suffix=timestamped_suffix)
        # print(f"Created new folder: {new_folder}")
        # print(timestamped_suffix)
    print('Done!')
    # plt.show()
    # plt.close()
