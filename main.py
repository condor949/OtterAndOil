#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Otter and Oil, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.
"""
import os
import argparse
import webbrowser

from lib import *
from spaces import *
from spaces.ParabolicSpace import Parabolic3DSpace
from vehicles import *
from controllers import *
from tools.dataStorage import *
from tools.randomPoints import *


# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
browser = 'chrome'  # browser for visualization of animated GIF


###############################################################################
# Main simulation loop
###############################################################################
if __name__ == '__main__':
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
    main_param.add_argument('--disable-animation', action='store_true', dest='not_animated', default=False,
                            help='enable picture only mode, disable animation')
    main_param.add_argument('-space', '--space-file', dest='space_filename', default='space.json', help='')
    main_param.add_argument('--cache-dir', dest='cache_dir', default='data', help='')
    # do not store in config file
    main_param.add_argument('-c', '--config-file', dest='config_filename', default='', help='')

    sym_param = parser.add_argument_group('simulation parameters')
    sym_param.add_argument('--space-type', metavar='space_type', default='', dest='space_type', type=str,
                           help='a keyword describing the type of curve defining the space of the concentration field.'
                                'In fact, the derivative of the concentration function at the boundary of the isoline depends on this. '
                                'It can be parabolic and gaussian')
    sym_param.add_argument('-raw', '--store-raw', action='store_true', dest='store_raw', default=False,
                           help='space, trajectory and intensity data are stored in vector format.'
                                'Data extension for analysis - npy')
    sym_param.add_argument('-N', '--number-samples', metavar='N', default=20000, dest='N', type=int,
                           help='number of samples')
    sym_param.add_argument('-sT', '--sample-time', metavar='sample_time', default=0.02, dest='sample_time', type=float,
                           help='sample time [seconds]')
    sym_param.add_argument('-C', '--cycles', metavar='cycles', default=1, type=int,
                           help='the number of repetitions of the experiment')
    sym_param.add_argument('-R', '--start-radius', metavar='radius', default=1, type=int, dest='radius',
                           help='the radius of the catamaran starting position (m)')
    sym_param.add_argument('-NC', '--number-catamarans', metavar='catamarans', default=1, type=int, dest='catamarans',
                           help='the number of catamaran models participating in the experiment')
    sym_param.add_argument('-GS','--grid-size', metavar='grid_size', default=500, dest='grid_size', help='number of 3D data points')
    sym_param.add_argument('-FPS','--frames-per-second',metavar='FPS', default=30, dest='FPS', help='frames per second (animated GIF)')

    veh_params = parser.add_argument_group('flow parameters')
    veh_params.add_argument('-V', '--v-current', metavar='V_current', default=0, dest='V_current', type=float,
                            help='current flow speed (m/s)')
    veh_params.add_argument('-bC', '--beta-current', metavar='beta_current', default=-30.0, dest='beta_current',
                            type=float, help='current direction (deg)')

    args = parser.parse_args()
    #print(vars(args))
    if not args.config_filename:
        arguments = Arguments(**vars(args))
    else:
        arguments = read_and_assign_parameters(args.config_filename)

    ###############################################################################
    # Vehicle constructors
    ###############################################################################
    # printSimInfo()

    vehicles = [otter(V_current=arguments.V_current, beta_current=arguments.beta_current) for _ in range(arguments.catamarans)]
    # printVehicleinfo(vehicle, args.sample_time, args.N)

    # plotVehicleStates(simTime, simData, 1)
    # plotControls(simTime, simData, vehicle, 2)
    # [simTime, simData] = simulate(args.N, args.sample_time, vehicle)
    # plot3D(simData, grid_size, FPS, filename, 3)

    """ Uncomment the line below for 3D animation in the web browser. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))
    if arguments.clean_cache:
        clean_data()
    if arguments.big_picture:
        print('BE CAREFUL THE BIG PICTURE MODE REQUIRES MORE MEMORY')
    start_points = [[-10, -10]]
    if arguments.space_type == 'parabolic':
        space = Parabolic3DSpace(grid_size=arguments.grid_size,
                                 shift_x=0,
                                 shift_y=0,
                                 shift_z=-10,
                                 space_filename=args.space_filename)
    else:
        space = Gaussian3DSpace(grid_size=arguments.grid_size,
                                shift_x=0,
                                shift_y=0,
                                shift_z=-10,
                                space_filename=args.space_filename)
        arguments.space_type = "gaussian"

    space.set_contour_points(tol=1)
    for i in range(arguments.cycles):
        if arguments.catamarans > 1:
            start_points = generate_random_points(arguments.radius, arguments.catamarans)
        timestamped_suffix: str = create_timestamped_suffix()
        timestamped_folder: str = create_timestamped_folder(arguments.space_type,
                                                            f"s{i + 1}",
                                                            timestamped_suffix=timestamped_suffix)
        controller = IntensityBasedController(timestamped_folder=timestamped_folder,
                                              timestamped_suffix=timestamped_suffix,
                                              starting_points=start_points,
                                              sample_time=arguments.sample_time,
                                              space=space)

        [simTime, swarmData] = simultaneous_simulate(vehicles=vehicles, initial_positions=start_points, N=arguments.N,
                                                     sample_time=arguments.sample_time, controller=controller)
        controller.plotting_intensity()
        controller.plotting_sigma()
        controller.plotting_quality(sample_time=arguments.sample_time)
        plotting_track(swarmData,
               arguments.grid_size,
               arguments.FPS,
               timestamped_folder,
               timestamped_suffix,
               space,
               arguments.big_picture,
               arguments.not_animated)
        space.plotting_surface(timestamped_folder, timestamped_suffix)

        arguments.store_in_config(folder=timestamped_folder,suffix=timestamped_suffix)
        space.store_in_config(timestamped_folder, timestamped_suffix)

    print('Done!')
