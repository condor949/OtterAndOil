# -*- coding: utf-8 -*-
"""
Simulator plotting functions:

plotVehicleStates(simTime, simData, figNo) 
plotControls(simTime, simData, vehicle, figNo)
def plot3D(simData, numDataPoints, FPS, filename, figNo)

Author:     Thor I. Fossen
"""

import math

import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
from lib.gnc import ssa
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from matplotlib import rc
from tools.random_generators import color_generator
from tools.dataStorage import *
from functools import partial
from spaces import BaseSpace

legendSize = 16
legendSize1 = 10
figSize1 = [25, 25]  # figure1 size in cm
bigFigSize1 = [50, 26]  # larger figure1 size in cm
figSize2 = [25, 13]  # figure2 size in cm
dpiValue = 150  # figure dpi value


def R2D(value):  # radians to degrees
    return value * 180 / math.pi


def cm2inch(value):  # inch to cm
    return value / 2.54


# plotVehicleStates(simTime, simData, figNo) plots the 6-DOF vehicle
# position/attitude and velocities versus time in figure no. figNo
def plotVehicleStates(simTime, swarmData, folder, suffix):
    # Time vector
    t = simTime

    rc('font', size=9)
    i = 0
    for simData in swarmData:
        # State vectors
        x = simData[:, 0]
        y = simData[:, 1]
        z = simData[:, 2]
        phi = R2D(ssa(simData[:, 3]))
        theta = R2D(ssa(simData[:, 4]))
        psi = R2D(ssa(simData[:, 5]))
        u = simData[:, 6]
        v = simData[:, 7]
        w = simData[:, 8]
        p = R2D(simData[:, 9])
        q = R2D(simData[:, 10])
        r = R2D(simData[:, 11])

        # Speed
        U = np.sqrt(np.multiply(u, u) + np.multiply(v, v) + np.multiply(w, w))

        beta_c = R2D(ssa(np.arctan2(v, u)))  # crab angle, beta_c
        alpha_c = R2D(ssa(np.arctan2(w, u)))  # flight path angle
        chi = R2D(ssa(simData[:, 5] + np.arctan2(v, u)))  # course angle, chi=psi+beta_c

        # Plots
        plt.figure(
            figsize=(cm2inch(figSize1[0]), cm2inch(figSize1[1])), dpi=dpiValue
        )
        # plt.grid()

        plt.subplot(3, 3, 1)
        plt.plot(y, x)
        plt.legend(["North-East positions,m"], fontsize=legendSize1)
        plt.grid()

        plt.subplot(3, 3, 2)
        # print(z.shape)
        z[0] = 0
        plt.plot(t, z)
        plt.legend(["Depth,m"], fontsize=legendSize1)
        plt.grid()

        plt.title("Vehicle states", fontsize=12)

        plt.subplot(3, 3, 3)
        phi[0] = 0
        theta[0] = 0
        plt.plot(t, phi, t, theta)
        plt.legend(["Roll angle,deg", "Pitch angle,deg"], fontsize=legendSize1)
        plt.grid()

        plt.subplot(3, 3, 4)
        plt.plot(t, U)
        plt.legend(["Speed,m/s"], fontsize=legendSize1)
        plt.grid()

        plt.subplot(3, 3, 5)
        plt.plot(t, chi)
        plt.legend(["Course angle,deg"], fontsize=legendSize1)
        plt.grid()

        plt.subplot(3, 3, 6)
        alpha_c[0] = 0
        theta[0] = 0
        plt.plot(t, theta, t, alpha_c)
        plt.legend(["Pitch angle,deg", "Flight path angle,deg"], fontsize=legendSize1)
        plt.grid()

        plt.subplot(3, 3, 7)
        plt.plot(t, u, t, v, t, w)
        plt.xlabel("Time,s", fontsize=12)
        plt.legend(
            ["Surge velocity,m/s", "Sway velocity,m/s", "Heave velocity,m/s"],
            fontsize=legendSize1,
        )
        plt.grid()

        plt.subplot(3, 3, 8)
        plt.plot(t, p, t, q, t, r)
        plt.xlabel("Time,s", fontsize=12)
        plt.legend(
            ["Roll rate,deg/s", "Pitch rate,deg/s", "Yaw rate,deg/s"],
            fontsize=legendSize1,
        )
        plt.grid()

        plt.subplot(3, 3, 9)
        plt.plot(t, psi, t, beta_c)
        plt.xlabel("Time,s", fontsize=12)
        plt.legend(["Yaw angle,deg", "Crab angle,deg"], fontsize=legendSize1)
        plt.grid()
        plt.savefig(os.path.join(folder,
                                 create_timestamped_filename_ext(f"states_v{i}",
                                                                 suffix,
                                                                 "png")))
        i+=1
        plt.close()

# plotControls(simTime, simData) plots the vehicle control inputs versus time
# in figure no. figNo
def plotControls(simTime, swarmData, vehicles, folder, suffix):
    DOF = 6

    # Time vector
    rc('font', size=10)
    t = simTime
    for vehicle in vehicles:
        plt.figure(figsize=(cm2inch(figSize2[0]), cm2inch(figSize2[1])), dpi=dpiValue)

        # Columns and rows needed to plot vehicle.dimU control inputs
        col = 2
        row = int(math.ceil(vehicle.dimU / col))

        # Plot the vehicle.dimU active control inputs
        for i in range(0, vehicle.dimU):

            u_control = swarmData[vehicle.serial_number][:, 2 * DOF + i]  # control input, commands
            u_actual = swarmData[vehicle.serial_number][:, 2 * DOF + vehicle.dimU + i]  # actual control input

            if vehicle.controls[i].find("deg") != -1:  # convert angles to deg
                u_control = R2D(u_control)
                u_actual = R2D(u_actual)

            plt.subplot(row, col, i + 1)
            plt.plot(t, u_control, t, u_actual)
            plt.legend(
                [vehicle.controls[i] + ", command", vehicle.controls[i] + ", actual"],
                fontsize=10,
            )
            plt.xlabel("Time,s", fontsize=12)
            plt.grid()
            plt.savefig(os.path.join(folder,
                                  create_timestamped_filename_ext(f"control_v{vehicle.serial_number}",
                                                                  suffix,
                                                                  "png")))
        plt.close()


# plot3D(simData,numDataPoints,FPS,filename,figNo) plots the vehicles position (x, y, z) in 3D
# in figure no. figNo
def plotting_track(swarmData, numDataPoints, FPS, folder, suffix, space: BaseSpace,
                   isolines: int = 0,
                   big_picture: bool=False,
                   not_animated: bool=False):
    # Attaching 3D axis to the figure
    if big_picture:
        fig = plt.figure(figsize=(cm2inch(bigFigSize1[0]), cm2inch(bigFigSize1[1])),
                         dpi=dpiValue)
    else:
        fig = plt.figure(figsize=(cm2inch(figSize1[0]), cm2inch(figSize1[1])),
                         dpi=dpiValue)

    rc('font', size=20)
    contour = plt.contour(space.get_X(), space.get_Y(), space.get_Z(), levels=isolines, cmap='viridis')  # Adjust `levels` to set number of isolines

    plt.clabel(contour, inline=True)  # Add labels to the isolines
    # plt.title('Track in the intensity field')
    plt.xlabel('X,m / East')
    plt.ylabel('Y,m / North')
    #plt.colorbar(contour, label='Intensity')  # Add a color bar for reference
    plt.contour(space.get_X(), space.get_Y(), space.get_Z(), levels=[space.target_isoline],
               colors='red')  # Intersection line
    #plt.show()

    # ax = p3.Axes3D(fig, auto_add_to_figure=False)
    # ax.view_init(elev=90, azim=-90)
    # fig.add_axes(ax)
    #
    # if show_intensity:
    #     # Plot intensity surface
    #     ax.plot_surface(space.get_X(), space.get_Y(), space.get_Z(), cmap='viridis', alpha=0.5)
    #
    # # Plot contour of target isoline
    # ax.contour(space.get_X(), space.get_Y(), space.get_Z(), zdir='z', offset=plane_z, levels=[plane_z], colors='red') # Intersection line
    # if zoning > 0:
        # step = int(np.max(space.get_Z()) / zoning)
        #print(np.max(space.get_Z()))
        # Example usage
        # colors = green_to_yellow(zoning)

        # plt.colorbar(contour, label='Z-value')
        # for i in range(1, zoning):
        #
        #     # Create the contour plot
        #     plt.figure(figsize=(8, 6))
        #     contour = plt.contour(X, Y, Z, levels=10, cmap='viridis')  # Adjust `levels` to set number of isolines
        #     plt.clabel(contour, inline=True, fontsize=10)  # Add labels to the isolines
        #     plt.title('Isoline Graph (Contour Plot)')
        #     plt.xlabel('X-axis')
        #     plt.ylabel('Y-axis')
        #     plt.colorbar(contour, label='Z-value')  # Add a color bar for reference
        #     ax.colorbar(contour, label='Z-value')  # Add a color bar for reference
        #     plt.show()
        #
        #     ax.contour(space.get_X(), space.get_Y(), space.get_Z(), zdir='z', offset=step, levels=[step*i], colors=colors[i-1])  # Intersection line
        # ax.contour(space.get_X(), space.get_Y(), space.get_Z(), zdir='z', offset=step, levels=[int(np.max(space.get_Z()))], colors=colors[zoning])  # Intersection line
    # Plot calculated contour points
    # ax.plot(np.array([point[0] for point in space.contour_points]), np.array([point[1] for point in space.contour_points]), color='green')
    # Plot contour of intensity area
    #ax.contour(space.get_X(), space.get_Y(), space.get_Z(), zdir='z', offset=plane_z, levels=[space.shift_xyz.shift_z()+1], colors='orange')

    # Animation function
    def anim_function(num, plotData):
        for line, dataSet in plotData.items():
            line.set_data(dataSet[0:2, :num])
            # line.set_3d_properties(dataSet[2, :num])
        return plotData.keys()

    color_gen = color_generator()
    plotData = {}
    i = 0
    for simData in swarmData:
        # State vectors
        x = simData[:, 0]
        y = simData[:, 1]
        z = simData[:, 2]
        # down-sampling the xyz data points
        N = y[::len(x) // numDataPoints];
        E = x[::len(x) // numDataPoints];
        D = z[::len(x) // numDataPoints];

        dataSet = np.array([N, E, -D])  # Down is negative z
        # Highlight the first point with an asterisk
        color = next(color_gen)
        plt.plot(dataSet[0][0], dataSet[1][0], marker='*', markersize=6, color=color, label='Start Point', zorder=10)
        # Line/trajectory plot
        line = plt.plot(dataSet[0], dataSet[1], lw=2, c=color, zorder=10, label=f'agent {i+1}')[0]
        plotData[line] = dataSet
        i+=1
    # Setting the axes properties
    # ax.set_xlabel('X,m / East')
    # ax.set_ylabel('Y,m / North')
   # ax.set_zlim3d([-10, 30])  # default depth = -100 m

    # if np.amax(z) > 100.0:
    #     ax.set_zlim3d([-np.amax(z), 20])

   # ax.set_zlabel('Intensity')
   # ax.set_zticks([])

    # Plot 2D surface for z = 0
    # [x_min, x_max] = ax.get_xlim()
    # [y_min, y_max] = ax.get_ylim()
    # indent = 0
    # x_grid = np.arange(x_min - indent, x_max + indent)
    # y_grid = np.arange(y_min - indent, y_max + indent)
    # [xx, yy] = np.meshgrid(x_grid, y_grid)
    # zz = 0 * xx
    # ax.plot_surface(xx, yy, zz, alpha=0.3)

    # Title of plot
    # ax.set_title('North-East-Down')

    plt.savefig(os.path.join(folder,
                             create_timestamped_filename_ext('track',
                                                             suffix,
                                                             "png")))
    #plt.show()

    if not not_animated:
        # Create the animation object
        # if isometric:
        #     ax.view_init(elev=30.0, azim=-120.0)
        ani = animation.FuncAnimation(fig,
                                      partial(anim_function, plotData=plotData),
                                      frames=numDataPoints,
                                      interval=200,
                                      blit=False,
                                      repeat=True)
        # Save the 3D animation as a gif file
        update_func = lambda _i, _n: progress_bar.update(1)
        with tqdm(total=getattr(ani, "_save_count"), desc="Animation Writing") as progress_bar:
            try:
                writer = animation.FFMpegWriter(fps=FPS)
            except:
                writer = animation.PillowWriter(fps=FPS)

            ani.save(os.path.join(folder,
                                  create_timestamped_filename_ext('track',
                                                                  suffix,
                                                                  "gif")), writer=writer, progress_callback=update_func)
    plt.close()
