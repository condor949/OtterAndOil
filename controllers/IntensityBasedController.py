from numpy.ma.core import cumsum

from spaces import BaseSpace
from .BaseController import BaseController
import numpy as np
import matplotlib.pyplot as plt
from tools.dataStorage import *
from tools.randomPoints import normalize


class IntensityBasedController(BaseController):
    def __init__(self, timestamped_folder, timestamped_suffix, vehicles, N, sample_time, space: BaseSpace, f0=0, mu=0.5):
        super().__init__()
        self.timestamped_folder = timestamped_folder
        self.timestamped_suffix = timestamped_suffix
        self.m_f_prev = [space.get_intensity(vehicle.starting_point[1], vehicle.starting_point[0]) for vehicle in vehicles]
        self.sample_time = sample_time
        self.f0 = f0
        self.mu = mu
        self.space = space
        self.number_of_vehicles = len(vehicles)
        self.intensity = np.zeros((self.number_of_vehicles, N), dtype=float)
        self.der = np.zeros((self.number_of_vehicles, N), dtype=float)
        self.mu_tanh = np.zeros((self.number_of_vehicles, N), dtype=float)
        self.sigmas = np.zeros((self.number_of_vehicles, N), dtype=float)
        self.quality_array = np.zeros((self.number_of_vehicles, N), dtype=float)


    def berman_law(self, vehicle, step, f_current, f_prev):
        self.der[vehicle, step] = (f_current - f_prev) / self.sample_time
        self.mu_tanh[vehicle, step] = self.mu * np.tanh(f_current - self.f0)
        self.sigmas[vehicle, step] = -np.sign(self.der[vehicle, step] + self.mu_tanh[vehicle, step])
        return self.sigmas[vehicle, step]


    def generate_control(self, vehicles, positions, step):
        m_f_current = [self.space.get_intensity(eta[1], eta[0]) for eta in positions]
        # self.quality_array.append([self.space.get_nearest_contour_point_norm(eta[0], eta[1]) for eta in positions])
        controls = []
        for vehicle in range(self.number_of_vehicles):
            f_current = m_f_current[vehicle]
            f_prev = self.m_f_prev[vehicle]
            sigma = self.berman_law(vehicle, step, f_current, f_prev)

            if sigma < 0:
                u_control = [vehicles[vehicle].n_min, vehicles[vehicle].n_max]
            elif sigma > 0:
                u_control = [vehicles[vehicle].n_max, vehicles[vehicle].n_min]
            else:
                u_control = [0, 0]
            controls.append(u_control)

            self.intensity[vehicle, step] = f_current
            self.quality_array[vehicle, step] = self.space.get_nearest_contour_point_norm(positions[vehicle][0], positions[vehicle][1])
        self.m_f_prev = m_f_current
        return controls


    def plotting_sigma(self):
        print(np.array(self.der).shape)
        print(np.array(self.mu_tanh).shape)
        print(np.array(self.sigmas).shape)
        result = np.array([normalize(der_vehicle, 10) for der_vehicle in self.der])
        print(result.shape)
        print(self.simTime.shape)
        print(np.array(self.simTime).shape)
        print(self.simTime.T[0].shape)
        print(self.simTime.T[0])
        print(result[0].shape)
        print(result[0])
        for vehicle in range(self.number_of_vehicles):
            plt.figure()
            plt.plot(self.simTime, result[vehicle], label='Reaction')
            plt.plot(self.simTime, self.mu_tanh[vehicle], label='Control')
            plt.hlines(0, xmin=0, xmax=np.max(self.simTime), color='gray') # 0 axis
            plt.xlabel('Time,s', fontsize=12)
            plt.ylabel('Sigma', fontsize=12)
            plt.title("Control",
                       fontsize=10)
            plt.legend()
            plt.savefig(os.path.join(self.timestamped_folder,
                                     create_timestamped_filename_ext(f'sigmas_v{vehicle}',
                                                                     self.timestamped_suffix,
                                                                     'png')))
        plt.close()


    def plotting_quality(self):
        cumulative_quality = cumsum(self.sample_time*self.quality_array, axis=1)
        print("quality")
        print(cumulative_quality.shape)

        plt.figure()
        plt.xlabel('Time,s', fontsize=12)
        plt.ylabel('Total integral', fontsize=12)
        plt.title('The total value of the shortest distance at each point of the trajectory',
                  fontsize=10)
        for i in range(self.number_of_vehicles):
            plt.plot(self.simTime, cumulative_quality[i], label=f'agent {i+1}')
        plt.legend()
        plt.savefig(os.path.join(self.timestamped_folder,
                                 create_timestamped_filename_ext('quality',
                                                                 self.timestamped_suffix,
                                                                 'png')))
        plt.close()


    def plotting_intensity(self, separate_plots=False):
        """
        Plots the field intensity based on the number of steps for multiple agents.

        Parameters:
            data (list of lists): Each sublist contains intensity values for an agent.
            separate_plots (bool): If True, plots each agent on a separate subplot.
                                   If False, plots all agents on a single plot.
        """
        num_agents = len(self.intensity)
        #steps = range(len(self.intensity[0]))  # Assume all agents have the same number of steps

        if separate_plots:
            # Create a grid of subplots for each agent
            fig, axes = plt.subplots(num_agents, 1, figsize=(8, 4 * num_agents), sharex=True)
            if num_agents == 1:
                axes = [axes]  # Make it iterable if there is only one subplot

            for i, (intensities, ax) in enumerate(zip(self.intensity, axes)):
                ax.plot(self.simTime, intensities, label=f'Agent {i + 1}', color=f'C{i}')
                ax.set_title(f'Agent {i + 1} Field Intensity Over Steps')
                ax.set_xlabel('Time,s')
                ax.set_ylabel('Field Intensity')
                ax.legend()

            plt.tight_layout()

        else:
            # Plot all agents on a single graph
            plt.figure(figsize=(10, 6))
            for i, intensities in enumerate(self.intensity):
                plt.plot(self.simTime, intensities, label=f'Agent {i + 1}', color=f'C{i}')

            plt.title('Field Intensity Over Steps for All Agents')
            plt.xlabel('Time,s')
            plt.ylabel('Field Intensity')
            plt.legend()
        plt.savefig(os.path.join(self.timestamped_folder,
                                 create_timestamped_filename_ext('intensity',
                                                                 self.timestamped_suffix,
                                                                 'png')))
        plt.close()
