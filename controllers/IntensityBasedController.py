import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import pi
from tqdm import tqdm
from matplotlib import rc
from spaces import BaseSpace
from functools import partial
from tools.dataStorage import *
from numpy.ma.core import cumsum
from .BaseController import BaseController
from tools.random_generators import normalize
from tools.random_generators import color_generator

legendSize = 16
legendSize1 = 10
figSize1 = [25, 25]  # figure1 size in cm
bigFigSize1 = [50, 26]  # larger figure1 size in cm
figSize2 = [25, 13]  # figure2 size in cm
dpiValue = 150  # figure dpi value

def R2D(value):  # radians to degrees
    return value * 180 / pi

def cm2inch(value):  # inch to cm
    return value / 2.54


class IntensityBasedController(BaseController):
    name = 'intensity'
    def __init__(self, vehicles, sim_time: int, sample_time: float, space: BaseSpace, FPS=30, isolines=10, f0=0, mu=0.5):
        super().__init__(vehicles, sim_time, sample_time, space)
        self.m_f_prev = [space.get_intensity(vehicle.starting_point[1], vehicle.starting_point[0]) for vehicle in vehicles]
        self.f0 = f0
        self.mu = mu
        self.FPS = FPS
        self.isolines = isolines
        self.intensity = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.der = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.mu_tanh = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.sigmas = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.quality_array = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.type = 'Individual intensity based controller'

    def __str__(self):
        return (f'---controller--------------------------------------------------------------------------\n'
                f'{self.type}\n'
                f'Sampling frequency: {round(1 / self.sample_time)} Hz\n'
                f'Sampling time: {self.sample_time} seconds\n'
                f'Simulation time: {round(self.sim_time)} seconds\n'
                f'Numbers of vehicles: {self.number_of_vehicles}')

    def berman_law(self, vehicle, step, f_current, f_prev):
        self.der[vehicle, step] = (f_current - f_prev) / self.sample_time
        self.mu_tanh[vehicle, step] = self.mu * np.tanh(f_current - self.f0)
        self.sigmas[vehicle, step] = -np.sign(self.der[vehicle, step] + self.mu_tanh[vehicle, step])
        return self.sigmas[vehicle, step]

    def generate_control(self, positions, step):
        m_f_current = [self.space.get_intensity(eta[1], eta[0]) for eta in positions]
        # self.quality_array.append([self.space.get_nearest_contour_point_norm(eta[0], eta[1]) for eta in positions])
        controls = []
        for vehicle in self.vehicles:
            f_current = m_f_current[vehicle.serial_number]
            #print(f_current)
            f_prev = self.m_f_prev[vehicle.serial_number]
            sigma = self.berman_law(vehicle.serial_number, step, f_current, f_prev)

            if sigma < 0:
                u_control = [vehicle.n_min, vehicle.n_max]
            elif sigma > 0:
                u_control = [vehicle.n_max, vehicle.n_min]
            else:
                u_control = [0, 0]
            controls.append(u_control)
            #print(u_control)
            self.intensity[vehicle.serial_number, step] = f_current
            self.quality_array[vehicle.serial_number, step] = self.space.get_nearest_contour_point_norm(positions[vehicle.serial_number][0], positions[vehicle.serial_number][1])

        self.m_f_prev = m_f_current
        return controls

    def plotting_sigma(self, store_plot=False, **arguments):
        # print(np.array(self.der).shape)
        # print(np.array(self.mu_tanh).shape)
        # print(np.array(self.sigmas).shape)
        result = np.array([normalize(der_vehicle, 10) for der_vehicle in self.der])

        for vehicle in range(self.number_of_vehicles):
            plt.figure()
            result[vehicle][0] = 0
            plt.plot(self.simTime, result[vehicle], label='Reaction')
            plt.plot(self.simTime, self.mu_tanh[vehicle], label='Control')
            plt.hlines(0, xmin=0, xmax=np.max(self.simTime), color='gray') # 0 axis
            plt.xlabel('Time,s', fontsize=12)
            plt.ylabel('Sigma', fontsize=12)
            plt.legend()
            if store_plot:
                plt.savefig(self.data_storage.get_path(f'sigmas_v{vehicle}','png'))
                plt.close()
            else:
                plt.title(f"Control v{vehicle}", fontsize=10)
                plt.show()

    def plotting_quality(self, store_plot=False, **arguments):
        cumulative_quality = cumsum(self.sample_time*self.quality_array, axis=1)

        plt.figure()
        plt.xlabel('Time,s', fontsize=12)
        plt.ylabel('Total integral', fontsize=12)
        plt.title('The total value of the shortest distance at each point of the trajectory',
                  fontsize=10)
        for i in range(self.number_of_vehicles):
            plt.plot(self.simTime, cumulative_quality[i], label=f'Agent {i+1}', color=self.colors.get(i))
        plt.legend()
        if store_plot:
            plt.savefig(self.data_storage.get_path('quality', 'png'))
            plt.close()
        else:
            plt.show()
        cumulative_quality.dump('quality.npy')

    def plotting_intensity(self, separate_plots=False, store_plot=False, **arguments):
        """
        Plots the field intensity based on the number of steps for multiple agents.

        Parameters:
            data (list of lists): Each sublist contains intensity values for an agent.
            separate_plots (bool): If True, plots each agent on a separate subplot.
                                   If False, plots all agents on a single plot.
        """
        if separate_plots:
            # Create a grid of subplots for each agent
            fig, axes = plt.subplots(self.number_of_vehicles, 1, figsize=(8, 4 * self.number_of_vehicles), sharex=True)
            if self.number_of_vehicles == 1:
                axes = [axes]  # Make it iterable if there is only one subplot

            for i, (intensities, ax) in enumerate(zip(self.intensity, axes)):
                ax.plot(self.simTime, intensities + self.space.target_isoline, label=f'Agent {i + 1}', color=self.colors.get(i))
                ax.set_xlabel('Time,s')
                ax.set_ylabel('Field Intensity')
                ax.legend()
                if not store_plot:
                    ax.set_title(f'Agent {i + 1} Field Intensity Over Steps')
            plt.tight_layout()
        else:
            # Plot all agents on a single graph
            plt.figure(figsize=(10, 6))
            for i, intensities in enumerate(self.intensity):
                plt.plot(self.simTime, intensities + self.space.target_isoline, label=f'Agent {i + 1}', color=self.colors.get(i))

            plt.xlabel('Time,s')
            plt.ylabel('Field Intensity')
            plt.legend()
        if store_plot:
            plt.savefig(self.data_storage.get_path('intensity', 'png'))
            plt.close()
        else:
            plt.title('Field Intensity Over Steps for All Agents')
            plt.show()
            plt.close()
        #self.intensity.dump(self.data_storage.get_path('intensity','npy'))


    def plotting_track(self, swarmData = None,
                       big_picture: bool = False,
                       not_animated: bool = False,
                       store_plot: bool = False,
                       **arguments):
        # Attaching 3D axis to the figure
        if big_picture:
            fig = plt.figure(figsize=(cm2inch(bigFigSize1[0]), cm2inch(bigFigSize1[1])),
                             dpi=dpiValue)
        else:
            fig = plt.figure(figsize=(cm2inch(figSize1[0]), cm2inch(figSize1[1])),
                             dpi=dpiValue)

        rc('font', size=20)
        contour = plt.contour(self.space.get_X(), self.space.get_Y(), self.space.get_Z(), levels=self.isolines,
                              cmap='viridis')  # Adjust `levels` to set number of isolines

        plt.clabel(contour, inline=True)  # Add labels to the isolines
        plt.xlabel('X,m / East')
        plt.ylabel('Y,m / North')
        # plt.colorbar(contour, label='Intensity')  # Add a color bar for reference
        plt.contour(self.space.get_X(), self.space.get_Y(), self.space.get_Z(), levels=[self.space.target_isoline],
                    colors='red')  # Intersection line

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
            N = y[::len(x) // self.space.grid_size];
            E = x[::len(x) // self.space.grid_size];
            D = z[::len(x) // self.space.grid_size];

            dataSet = np.array([N, E, -D])  # Down is negative z
            # Highlight the first point with an asterisk
            color = next(color_gen)
            plt.plot(dataSet[0][0], dataSet[1][0], marker='*', markersize=6, color=color, label='Start Point',
                     zorder=10)
            # Line/trajectory plot
            line = plt.plot(dataSet[0], dataSet[1], lw=2, c=color, zorder=10, label=f'agent {i + 1}')[0]
            plotData[line] = dataSet
            i += 1

        if store_plot:
            plt.savefig(self.data_storage.get_path('track', "png"))
        else:
            not_animated = True
            plt.title('Track in the intensity field')
            plt.show()

        if not not_animated:
            # Create the animation object
            # if isometric:
            #     ax.view_init(elev=30.0, azim=-120.0)
            ani = animation.FuncAnimation(fig,
                                          partial(anim_function, plotData=plotData),
                                          frames=self.space.grid_size,
                                          interval=200,
                                          blit=False,
                                          repeat=True)
            # Save the 3D animation as a gif file
            update_func = lambda _i, _n: progress_bar.update(1)
            with tqdm(total=getattr(ani, "_save_count"), desc="Animation Writing") as progress_bar:
                try:
                    writer = animation.FFMpegWriter(fps=self.FPS)
                except:
                    writer = animation.PillowWriter(fps=self.FPS)

                ani.save(self.data_storage.get_path('track', "gif"),
                         writer=writer,
                         progress_callback=update_func)

        plt.close()
