#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Otter and Oil, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.
"""
import argparse
import spaces as sp
import vehicles as vs
import controllers as cs

from lib import *
from tools import *

#("░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░\n"
# "░░░░░░░░░░░░░░░░░▒▓▒▒░░▒▓▒░░░░░░░░░░░░░░░░░░░░░░░\n"
# "░░░░░░░▓▓▒▒░░░░▒▓▓▓▓▓▓█▓▒▒▓░░░░░░░░░░░░░░░░░░░░░░\n"
# "░░░░░░░▒▓░░░░░░░░░░▓█▓▒▒▒▒▒▓▓░░░░░░░░░░░░░░░░░░░░\n"
# "░░░░░░░░░█▓▓▓░░░░░░██▓░░░░░░░▒▒▒░░░░░░░░░░░░░░░░░\n"
# "░░▒▓▒▒░░░█▓▓▓▒░░░░▒█▓█▒░░░░▒▒░░░░░░▒▓▒▒░░░░░░░░░░\n"
# "░░█████▓█▓▓▒▓▓▓▓▓█▓▒██▓░░░░░░░░░░░▓█████▓░░░░░░░▓\n"
# "░░░░▓███▓▓▒▒▒▒▒▒▓▓▓▓▓▓▓▒░░░░░░░░▒███████▒░░░░░░░░\n"
# "░▒░░░▒▓████▓▓▒▒▒▒▒▓▓▓▓▓▓▓▓▒▒▒▓▒██████████▓▓████▒░\n"
# "░▒░▒▒░▒▒▓██████▓▓▓▓▓▓▓▓▓▓▓▓▓▓███████████████████░\n"
# "░░▒░░░░░░░▓████████▓▓▓▓▓▓▓▓▓▓▓█░░░░░░░░░░░░░░░░░░\n"
# "░░░░░░░░░░░░▒▓█████████████▓▓▓█▓░░░░░░░░░░░░░░░░░\n"
# "░░░░░░░░░░░░░░░░▒▓█████████████▓▓█▓▒░░░░░░░░░░░░░\n"
# "░░░░░░░░░░░░░░░░░░░░▒▓███████████████▓▓░░░░░░░░░░\n"
# "░░░░░░░░░░░░░░░░░░░░░░░░░▒▓▓████████████░░░░░░░░░\n"
# "░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▒▓▓▓▓▓░░░░░░░░░\n"
# "░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░\n"
# "░░░░░░░░▓██████▒░░▓██░░░▒██░░░░░░░░░░░░░░░░░░░░░░\n"
# "░░░░░░░██▓░░░▒██▒▓████▓▓█████░░▓████▒░░▓██▓█▓░░░░\n"
# "░░░░░░░██▒░░░░▓█▒░▓██░░░▒██░░░██▓░░▓█▒░▓██░░░░░░░\n"
# "░░░░░░░███░░░▒██▒░▓██░░░▒██░░░██▓░░░░░░▓██░░░░░░░\n"
# "░░░░░░░░▓█████▓░░░░███▓░░▓███░░██████▒░▓██░░░░░░░\n"
# "░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░\n")
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
    # do not store in config file
    main_param.add_argument('-c', '--config-file', dest='config_filename', default='', help='')
    args = parser.parse_args()

    arguments = read_and_assign_arguments(args.config_filename)

    ###############################################################################
    # Vehicle constructors
    ###############################################################################
    if arguments.vehicles == 1:
        starting_points = [[0, 0]]
    elif len(arguments.start_points) == arguments.vehicles:
        starting_points = arguments.start_points
    else:
        starting_points = [next(point_generator(arguments.radius, arguments.vehicles)) for _ in range(arguments.vehicles)]
    vehicles = []
    ng = number_generator()
    cg = color_generator()
    for vehicle_name in arguments.vehicle_types:
        for order_number in range(arguments.vehicles):
            vehicles.append(vs.create_instance(vehicle_name,
                                               V_current=arguments.V_current,
                                               serial_number=next(ng),
                                               shift=arguments.shift_vehicle,
                                               color=next(cg),
                                               starting_point=starting_points[order_number]))
    for vehicle in vehicles:
        print(vehicle)

    if arguments.clean_cache:
        clean_data()
    if arguments.big_picture:
        print('BE CAREFUL THE BIG PICTURE MODE REQUIRES MORE MEMORY')
    space = sp.create_instance(arguments.peak_type,
                               x_range=(-arguments.axis_abs_max, arguments.axis_abs_max),
                               y_range=(-arguments.axis_abs_max, arguments.axis_abs_max),
                               grid_size=arguments.grid_size,
                               shift_xyz=arguments.shift_xyz,
                               space_filename=arguments.peaks_filename,
                               target_isoline=arguments.target_isoline)
    space.set_contour_points(tol=1)
    print(space)

    for i in range(arguments.cycles):
        data_storage = DataStorage(space.type, i, arguments.cache_dir)

        space.set_data_storage(data_storage)
        plotting_all(space,
                    store_plot=arguments.store_plot)

        controller = cs.create_instance(arguments.controller_type,
                                        vehicles=vehicles,
                                        sim_time=arguments.sim_time_sec,
                                        sample_time=arguments.sample_time,
                                        space=space,
                                        FPS=arguments.FPS,
                                        isolines=arguments.isolines,
                                        eps=arguments.eps,
                                        e_max_cap=arguments.error_max_cap,
                                        dynamic_error_max=arguments.dynamic_error_max,
                                        smoothing=arguments.smoothing,
                                        plot_config_path=arguments.plot_config,
                                        use_latex=getattr(arguments, 'use_latex', True))
        controller.set_data_storage(data_storage)
        print(controller)
        print(data_storage)

        swarmData = simultaneous_simulate(controller=controller)
        # np.array(controller.u_controls).dump('u_controls.npy')
        np.array(controller.nus).dump('nus.npy')
        np.array(controller.dss).dump('dss.npy')
        np.array(controller.n_rots).dump('n_forwards.npy')
        np.array(controller.times_outside).dump('times_outside.npy')
        # plotting_all(controller,
        #             separating_plots=arguments.separating_plots,
        #             not_animated=arguments.not_animated,
        #             isometric=arguments.isometric,
        #             store_plot=arguments.store_plot,
        #             big_picture=arguments.big_picture,
        #             swarmData=swarmData)
        controller.plotting_intensity(x=controller.simTime, y=controller.intensity, store_plot=arguments.store_plot, for_publication=arguments.for_publication, colors=controller.colors)
        controller.plotting_error(x=controller.simTime, y=controller.errors_norm, store_plot=arguments.store_plot, for_publication=arguments.for_publication, colors=controller.colors)
        cumulative_mean = np.cumsum(controller.errors_norm, axis=1) / (np.arange(len(controller.errors_norm[0])) + 1)
        #cumulative_mean = cumulative_mean.T
        controller.plotting_error_avg(x=controller.simTime, y=cumulative_mean, store_plot=arguments.store_plot, for_publication=arguments.for_publication, colors=controller.colors)
        # controller.plotting_sigma()
        # controller.plotting_quality()
        controller.plotting_track(swarmData,
                                  arguments.big_picture,
                                  arguments.not_animated,
                                  arguments.store_plot)

        arguments.set_data_storage(data_storage)
        arguments.store_in_config()
        space.store_in_config()
        for i, error_sum in enumerate(controller.sum_error_values):
            print(f'vihicle {i}: e_norm = {error_sum / controller.sim_time}')
            print(controller.e_max)

    print('Done!')
