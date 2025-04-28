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
from controllers import IntensityBasedController

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
        data_storage = DataStorage(space.type, i)

        space.set_data_storage(data_storage)
        #plotting_all(space,
        #             store_plot=arguments.store_plot)

        controller = cs.create_instance(arguments.controller_type,
                                        vehicles=vehicles,
                                        sim_time=arguments.sim_time_sec,
                                        sample_time=arguments.sample_time,
                                        space=space,
                                        FPS=arguments.FPS,
                                        isolines=arguments.isolines)
        controller.set_data_storage(data_storage)
        print(controller)
        print(data_storage)


        def return_ds(self, nu):
            #return 1
            return np.sqrt(nu[0] ** 2 + nu[1] ** 2)


        IntensityBasedController.return_ds = return_ds


        def return_n_rot(self, sigma):
            #return 0
            return 30 * sigma


        IntensityBasedController.return_n_rot = return_n_rot


        def return_n_forward(self, f_current):
            #return 0
            return 100 - 80 * np.exp(-0.01 * (f_current - self.f0) ** 2)


        IntensityBasedController.return_n_forward = return_n_forward


        def return_u_control(self, sigma, n_min, n_max, n_rot, n_forward):
            if sigma < 0:
                # return [vehicle.n_min, vehicle.n_max]
                return [n_rot - n_forward, n_forward + n_rot]
            elif sigma > 0:
                # return [vehicle.n_max, vehicle.n_min]
                return [n_forward + n_rot, n_rot - n_forward]
            else:
                return [0, 0]


        IntensityBasedController.return_u_control = return_u_control

        swarmData = simultaneous_simulate(controller=controller)
        np.array(controller.u_controls).dump('u_controls.npy')
        np.array(controller.nus).dump('nus.npy')
        np.array(controller.dss).dump('dss.npy')
        np.array(controller.n_rots).dump('n_forwards.npy')
        plotting_all(controller,
                    separating_plots=arguments.separating_plots,
                    not_animated=arguments.not_animated,
                    isometric=arguments.isometric,
                    store_plot=arguments.store_plot,
                    big_picture=arguments.big_picture,
                    swarmData=swarmData)
        # controller.plotting_intensity()
        # controller.plotting_sigma()
        # controller.plotting_quality()
        # controller.plotting_track(swarmData,
        #                             arguments.big_picture,
        #                             arguments.not_animated)

        # arguments.set_data_storage(data_storage)
        # arguments.store_in_config()
        # space.store_in_config()

    print('Done!')
