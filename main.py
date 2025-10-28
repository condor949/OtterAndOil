#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Otter and Oil, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.
"""
import argparse
import logging
import os
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed

import numpy as np

import spaces as sp
import vehicles as vs
from controllers.manager import ControllerManager
from controllers.plotter import ControllerPlotter, PlotRenderOptions, TrackRenderOptions
from controllers.plot_jobs import TrackJob, run_track_render

from lib import *
from tools import *

WORK_THRESHOLD = 3000


def _controller_label(idx, controller):
    name = getattr(controller, "name", "")
    abbreviation = getattr(controller, "abbreviation", "")
    base = name or abbreviation
    if base:
        return f"{base}#{idx + 1}"
    return f"controller #{idx + 1}"

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

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    logger = logging.getLogger("otter.main")

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
        logger.info("Starting cycle %d/%d", i + 1, arguments.cycles)
        data_storage = DataStorage(space.type, i, arguments.cache_dir)

        space.set_data_storage(data_storage)
        plotting_all(space,
                    store_plot=arguments.store_plot)

        controller_runs = controller_manager.initialize_controllers(space=space,
                                                                    data_storage=data_storage)
        controllers_only = [controller for controller, _ in controller_runs]
        controller_labels = {idx: _controller_label(idx, controller) for idx, controller in enumerate(controllers_only)}

        max_N = max((getattr(controller, "N", 0) for controller in controllers_only), default=0)
        total_controllers = len(controllers_only)
        multi = total_controllers > 1
        use_threads_for_sim = multi and (max_N >= WORK_THRESHOLD)

        if total_controllers:
            if use_threads_for_sim:
                logger.info("Starting threaded simulation for %d controllers (max N=%d)", total_controllers, max_N)
            else:
                logger.info("Starting sequential simulation for %d controllers (max N=%d)", total_controllers, max_N)

        def _simulate_one(idx, controller):
            controller.simultaneous_simulate()
            return idx

        if use_threads_for_sim:
            thr_workers = min(len(controllers_only), max(1, os.cpu_count() or 1))
            logger.info("Using %d worker threads for simulation", thr_workers)
            with ThreadPoolExecutor(max_workers=thr_workers) as pool:
                futures = {}
                for idx, controller in enumerate(controllers_only):
                    label = controller_labels.get(idx, f"controller #{idx + 1}")
                    logger.info("Dispatching simulation for %s", label)
                    futures[pool.submit(_simulate_one, idx, controller)] = idx
                logger.info("Waiting for %d simulation task(s) to complete...", len(futures))
                for future in as_completed(futures):
                    idx = future.result()
                    controller_manager.set_run_result(idx)
                    label = controller_labels.get(idx, f"controller #{idx + 1}")
                    logger.info("Simulation finished for %s (%d/%d)", label, idx + 1, total_controllers)
        else:
            for idx, controller in enumerate(controllers_only):
                label = controller_labels.get(idx, f"controller #{idx + 1}")
                logger.info("Simulating %s (%d/%d)...", label, idx + 1, total_controllers)
                idx = _simulate_one(idx, controller)
                controller_manager.set_run_result(idx)
                logger.info("Simulation finished for %s", label)

        if total_controllers:
            logger.info("Simulation phase completed")

        controller_runs = list(controller_manager.controller_runs)

            # for attribute_name, filename in (("nus", "nus"),
            #                                  ("dss", "dss"),
            #                                  ("n_rots", "n_forwards"),
            #                                  ("times_outside", "times_outside")):
            #     if hasattr(controller, attribute_name):
            #         np.array(getattr(controller, attribute_name)).dump(controller_storage.get_path(filename, 'npy'))
            #


        controllers_only = [controller for controller, _ in controller_runs]
        controller_labels = {idx: _controller_label(idx, controller) for idx, controller in enumerate(controllers_only)}
        plotter = ControllerPlotter(controllers_only,
                                    plot_config_path=arguments.plot_config,
                                    use_latex=getattr(arguments, 'use_latex', True),
                                    plot_options=PlotRenderOptions(big_picture=arguments.big_picture,
                                                                   separate_plots=getattr(arguments, 'separating_plots', False),
                                                                   store_plots=arguments.store_plot,
                                                                   for_publication=arguments.for_publication),
                                    track_options=TrackRenderOptions(big_picture=arguments.big_picture,
                                                                     not_animated=arguments.not_animated,
                                                                     store_plot=arguments.store_plot),
                                    data_storage=data_storage)

        for controller, _ in controller_runs:
            controller.errors_avg = np.cumsum(controller.errors_norm, axis=1) / (
                np.arange(controller.errors_norm.shape[1]) + 1
            )

        animate_enabled = not arguments.not_animated
        do_animation = arguments.store_plot and animate_enabled

        if do_animation:
            logger.info("Track animation rendering enabled; preparing jobs")
            jobs = []
            for idx, (controller, sim_data) in enumerate(controller_runs):
                label = controller_labels.get(idx, f"controller #{idx + 1}")
                logger.info("Queueing track render for %s", label)
                jobs.append(TrackJob(
                    snapshot=controller.track_snapshot(),
                    sim_data=sim_data,
                    out_png=controller.get_plot_path('track', 'png'),
                    out_gif=controller.get_plot_path('track', 'gif'),
                    dpi=plotter._dpi,
                    big_picture=plotter._track_options.big_picture,
                    use_latex=getattr(arguments, 'use_latex', True),
                    animate=animate_enabled,
                    title="Track in the intensity field",
                    label=label,
                ))

            if jobs:
                proc_workers = min(len(jobs), max(1, os.cpu_count() or 1))
                logger.info("Spawning %d process(es) for %d track job(s)", proc_workers, len(jobs))
                with ProcessPoolExecutor(max_workers=proc_workers) as pool:
                    futures = {pool.submit(run_track_render, job): job.label or controller_labels.get(i, f"controller #{i + 1}")
                               for i, job in enumerate(jobs)}
                    logger.info("Waiting for track rendering jobs to complete...")
                    for future in as_completed(futures):
                        label = futures[future]
                        future.result()
                        logger.info("Track rendering completed for %s", label)
                logger.info("All track rendering jobs finished")
            else:
                logger.info("No track rendering jobs were created")
        else:
            logger.info("Track animation disabled or not storing; rendering sequentially in main process")
            for idx, (controller, sim_data) in enumerate(controller_runs):
                label = controller_labels.get(idx, f"controller #{idx + 1}")
                logger.info("Rendering track for %s in main process", label)
                plotter.plotting_track(controller,
                                       sim_data)
                logger.info("Track rendering finished for %s", label)
            logger.info("Track rendering in main process finished")

        logger.info("Rendering intensity and error plots")
        plotter.plotting_intensity()
        plotter.plotting_error()
        plotter.plotting_error_avg()
        plotter.plotting_error_avg(combine=True)
        logger.info("Finished rendering intensity and error plots")

        arguments.set_data_storage(data_storage)
        arguments.store_in_config()
        space.store_in_config()
        for controller_index, (controller, _) in enumerate(controller_runs):
            for vehicle_index, error_sum in enumerate(controller.sum_error_values):
                print(f'controller {controller_index + 1} vehicle {vehicle_index}: e_norm = {error_sum / controller.sim_time}')
            print(controller.e_max)

        logger.info("Cycle %d/%d completed", i + 1, arguments.cycles)

    logger.info("All cycles completed")
    print('Done!')
