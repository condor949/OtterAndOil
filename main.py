#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Otter and Oil, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.
"""
import argparse
import spaces as sp
import vehicles as vs
from controllers.manager import ControllerManager

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
    controller_vehicle_pairs = ControllerVehiclePairs(arguments.controller_vehicle_pairs)

    ###############################################################################
    # Vehicle constructors
    ###############################################################################
    global_color_generator = color_generator()
    serial_numbers = number_generator()
    controller_vehicle_groups = []

    for assignment in controller_vehicle_pairs:
        vehicles = []
        #print(assignment)
        for vehicle_config in assignment.vehicles:
            starting_point = vehicle_config.start_point
            vehicles.append(vs.create_instance(vehicle_config.vehicle_type,
                                               V_current=arguments.V_current,
                                               serial_number=next(serial_numbers),
                                               shift=getattr(arguments, 'shift_vehicle', None),
                                               color=next(global_color_generator),
                                               starting_point=starting_point))
        controller_vehicle_groups.append((assignment, vehicles))

    for _, vehicles in controller_vehicle_groups:
        for vehicle in vehicles:
            print(vehicle)

    controller_manager = ControllerManager(controller_vehicle_groups, arguments)

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

        controller_runs = controller_manager.initialize_controllers(space=space,
                                                                    data_storage=data_storage)
        swarm_data = []

        for i, (controller, _) in enumerate(controller_runs):
            result = simultaneous_simulate(controller=controller)
            controller_runs[i] = (controller, result)
            controller_manager.set_run_result(i, result)
            swarm_data.append(result)

            # for attribute_name, filename in (("nus", "nus"),
            #                                  ("dss", "dss"),
            #                                  ("n_rots", "n_forwards"),
            #                                  ("times_outside", "times_outside")):
            #     if hasattr(controller, attribute_name):
            #         np.array(getattr(controller, attribute_name)).dump(controller_storage.get_path(filename, 'npy'))
            #


        for controller, sim_data in controller_runs:
            controller.plotting_intensity(x=controller.simTime,
                                          y=controller.intensity,
                                          store_plot=arguments.store_plot,
                                          for_publication=arguments.for_publication,
                                          colors=controller.colors)
            controller.plotting_error(x=controller.simTime,
                                      y=controller.errors_norm,
                                      store_plot=arguments.store_plot,
                                      for_publication=arguments.for_publication,
                                      colors=controller.colors)
            cumulative_mean = np.cumsum(controller.errors_norm, axis=1) / (np.arange(len(controller.errors_norm[0])) + 1)
            controller.plotting_error_avg(x=controller.simTime,
                                          y=cumulative_mean,
                                          store_plot=arguments.store_plot,
                                          for_publication=arguments.for_publication,
                                          colors=controller.colors)
            controller.plotting_track(sim_data,
                                      arguments.big_picture,
                                      arguments.not_animated,
                                      arguments.store_plot)

        arguments.set_data_storage(data_storage)
        arguments.store_in_config()
        space.store_in_config()
        for controller_index, (controller, _) in enumerate(controller_runs):
            for vehicle_index, error_sum in enumerate(controller.sum_error_values):
                print(f'controller {controller_index + 1} vehicle {vehicle_index}: e_norm = {error_sum / controller.sim_time}')
            print(controller.e_max)

    print('Done!')
