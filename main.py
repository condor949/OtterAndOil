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
browser = 'chrome'  # browser for visualization of animated GIF

class Arguments:
    def __init__(self,
                 clean_cache: bool,
                 big_picture: bool,
                 not_animated: bool,
                 space_filename: str,
                 N: int,
                 sample_time: float,
                 cycles: int,
                 radius: int,
                 catamarans: int,
                 grid_size: int,
                 FPS: int,
                 V_current: float,
                 beta_current: float):
        self.clean_cache = clean_cache
        self.big_picture = big_picture
        self.not_animated = not_animated
        self.space_filename = space_filename
        self.N = N
        self.sample_time = sample_time
        self.cycles = cycles
        self.radius = radius
        self.catamarans = catamarans
        self.grid_size = grid_size
        self.FPS = FPS
        self.V_current = V_current
        self.beta_current = beta_current

###############################################################################
# Main simulation loop 
###############################################################################
def read_and_assign_parameters(input_filename):
    with open(input_filename, 'r') as file:
        data = json.load(file)

    # Assign values with appropriate types
    return Arguments(clean_cache = bool(data['clean_cache']),
                     big_picture = bool(data['big_picture']),
                     not_animated = bool(data['not_animated']),
                     space_filename = str(data['space_filename']),
                     N = int(data['N']),
                     sample_time = float(data['sample_time']),
                     cycles = int(data['cycles']),
                     radius = int(data['radius']),
                     catamarans = int(data['catamarans']),
                     grid_size=int(data['grid_size']),
                     FPS=int(data['FPS']),
                     V_current = float(data['V_current']),
                     beta_current = float(data['beta_current']))


# Save the variables to a new JSON file
def save_parameters(output_filename, arguments: Arguments):
    json_data = {
        "clean_cache": arguments.clean_cache,
        "big_picture": arguments.big_picture,
        "not_animated": arguments.not_animated,
        "space_filename": arguments.space_filename,
        "N": arguments.N,
        "sample_time": arguments.sample_time,
        "cycles": arguments.cycles,
        "radius": arguments.radius,
        "catamarans": arguments.catamarans,
        "grid_size": arguments.grid_size,
        "FPS": arguments.FPS,
        "V_current": arguments.V_current,
        "beta_current": arguments.beta_current
    }

    with open(output_filename, 'w') as saved_config:
        json.dump(json_data, saved_config, indent=4)


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
    main_param.add_argument('-disanim', '--disable-animation', action='store_true', dest='not_animated', default=False,
                            help='enable picture only mode, disable animation')
    main_param.add_argument('-space', '--space-file', dest='space_filename', default='./space.json', help='')
    main_param.add_argument('-c', '--config-file', dest='config_filename', default='./config.json', help='')

    sym_param = parser.add_argument_group('simulation parameters')
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

    veh_params = parser.add_argument_group('catamaran parameters')
    veh_params.add_argument('-V', '--v-current', metavar='V_current', default=0, dest='V_current', type=float,
                            help='current speed (m/s)')
    veh_params.add_argument('-bC', '--beta-current', metavar='beta_current', default=-30.0, dest='beta_current',
                            type=float, help='current direction (deg)')

    args = parser.parse_args()
    if not args.config_filename:
        arguments = Arguments(clean_cache = args.clean_cache,
                              big_picture = args.big_picture,
                              not_animated = args.not_animated,
                              space_filename = args.space_filename,
                              N = args.N,
                              sample_time = args.sample_time,
                              cycles = args.cycles,
                              radius = args.radius,
                              catamarans = args.catamarans,
                              grid_size=args.grid_size,
                              FPS=args.FPS,
                              V_current = args.V_current,
                              beta_current = args.beta_current)
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
    start_points = [[0, 0]]
    space = Gaussian3DSpace(grid_size=arguments.grid_size,shift_x=11,shift_y=11,shift_z=-10)
    space.add_gaussian_peak(x0=0, y0=0, amplitude=30, sigma_x=6, sigma_y=6)
    space.add_gaussian_peak(x0=5, y0=5, amplitude=25, sigma_x=1.5, sigma_y=4)
    space.add_gaussian_peak(x0=-5, y0=-5, amplitude=20, sigma_x=3, sigma_y=2)
    space.add_gaussian_peak(x0=-3, y0=6, amplitude=28, sigma_x=1.5, sigma_y=1.5)
    space.add_gaussian_peak(x0=4, y0=-6, amplitude=12, sigma_x=1.8, sigma_y=1)
    space.drown()
    space.set_contour_points(plane_z=0, tol=1)
    # print(space.contour_points)
    for i in range(arguments.cycles):
        if arguments.catamarans > 1:
            start_points = generate_random_points(arguments.radius, arguments.catamarans)
        controller = IntensityBasedController(starting_points=start_points, sample_time=arguments.sample_time, space=space)
        timestamped_suffix: str = create_timestamped_suffix()
        timestamped_folder: str = create_timestamped_folder(suffix=timestamped_suffix)
        # swarmData = []
        # for point in start_points:
        #     [simTime, simData] = simulate(point[0], point[1], args.N, args.sample_time, vehicle)
        #     swarmData.append(simData)

        [simTime, swarmData] = simultaneous_simulate(vehicles=vehicles, initial_positions=start_points, N=args.N,
                                                     sample_time=arguments.sample_time, controller=controller)
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
                                        sample_time=arguments.sample_time)
        plot3D(swarmData, arguments.grid_size, arguments.FPS, os.path.join(timestamped_folder,
                                                             create_timestamped_filename_ext('3D_animation',
                                                                                             timestamped_suffix,
                                                                                             "gif")),
                 1, arguments.big_picture, space, arguments.not_animated)
        # # new_folder = create_timestamped_folder(suffix=timestamped_suffix)
        # print(f"Created new folder: {new_folder}")
        # print(timestamped_suffix)
    print('Done!')
    # plt.show()
    # plt.close()
