from fontTools.ttLib.standardGlyphOrder import standardGlyphOrder
from numpy.ma.core import cumsum, array
from scipy.integrate import cumulative_trapezoid

from spaces import BaseSpace
from .BaseController import BaseController
import numpy as np


def oil_intensity(x, y):
    # print(-(x - 10) ** 2 - (y - 10) ** 2 + 30)
    return -(x - 10) ** 2 - (y - 10) ** 2 + 30


class IntensityBasedController(BaseController):

    def __init__(self, starting_points, sample_time, space: BaseSpace, f0=0, mu=1):
        super().__init__()
        #self.m_f_prev = [oil_intensity(point[0], point[1]) for point in starting_points]
        self.m_f_prev = [space.get_intensity(point[0], point[1]) for point in starting_points]
        self.sample_time = sample_time
        self.f0 = f0
        self.mu = mu*0.5
        self.space = space
        self.intensity = list()
        self.der = list()
        self.mu_tanh = list()
        self.sigmas = list()
        self.quality_array = list()

    def berman_law(self, f_current, f_prev):
        sigma = -np.sign((f_current - f_prev) / self.sample_time + self.mu * np.tanh(f_current - self.f0))

        self.der.append((f_current - f_prev) / self.sample_time)
        self.mu_tanh.append(self.mu * np.tanh(f_current - self.f0))
        self.sigmas.append(sigma)
        return sigma

    def generate_control(self, vehicles, positions):
        m_f_current = [self.space.get_intensity(eta[0], eta[1]) for eta in positions]
        #m_f_current = [oil_intensity(eta[0], eta[1]) for eta in positions]
        self.quality_array.append([self.space.get_nearest_contour_point_norm(eta[0], eta[1]) for eta in positions])
        controls = []
        for k in range(len(vehicles)):

            f_current = m_f_current[k]
            f_prev = self.m_f_prev[k]
            self.store_intensity(k,f_current)
            sigma = self.berman_law(f_current, f_prev)

            if sigma < 0:
                u_control = [vehicles[k].n_min, vehicles[k].n_max]
            elif sigma > 0:
                u_control = [vehicles[k].n_max, vehicles[k].n_min]
            else:
                u_control = [0, 0]
            controls.append(u_control)
        self.m_f_prev = m_f_current
        return controls

    def store_intensity(self, vehicle: int, intensity: float):
        if  vehicle >= len(self.intensity):
            self.intensity.append(list())
        self.intensity[vehicle].append(intensity)

    def create_sigma_graph(self, path):
        import matplotlib.pyplot as plt

        #plt.plot(self.sigmas, label="Sigma")
        plt.plot(self.der, label="Der")
        plt.plot(self.mu_tanh, label="Mu х Tanh")
        plt.legend()
        plt.savefig(path)
        plt.close()


    def create_quality_graph(self, path, sample_time):
        cumulative_quality = cumsum(sample_time*array(self.quality_array), axis=0)
        print(cumulative_quality.shape)
        import matplotlib.pyplot as plt

        #plt.plot(self.sigmas, label="Sigma")
        for i in range(cumulative_quality.shape[1]):
            plt.plot(cumulative_quality[:, i], label=f"Quality {i+1}")
        plt.legend()
        plt.savefig(path)
        plt.close()


    def create_intensity_graph(self, path, separate_plots=False):
        import matplotlib.pyplot as plt

        """
        Plots the field intensity based on the number of steps for multiple agents.

        Parameters:
            data (list of lists): Each sublist contains intensity values for an agent.
            separate_plots (bool): If True, plots each agent on a separate subplot.
                                   If False, plots all agents on a single plot.
        """
        num_agents = len(self.intensity)
        steps = range(len(self.intensity[0]))  # Assume all agents have the same number of steps
        # print(num_agents)
        # print(steps)
        # print()
        if separate_plots:
            # Create a grid of subplots for each agent
            fig, axes = plt.subplots(num_agents, 1, figsize=(8, 4 * num_agents), sharex=True)
            if num_agents == 1:
                axes = [axes]  # Make it iterable if there is only one subplot

            for i, (intensities, ax) in enumerate(zip(self.intensity, axes)):
                ax.plot(steps, intensities, label=f'Agent {i + 1}', color=f'C{i}')
                ax.set_title(f'Agent {i + 1} Field Intensity Over Steps')
                ax.set_xlabel('Steps')
                ax.set_ylabel('Field Intensity')
                ax.legend()

            plt.tight_layout()

        else:
            # Plot all agents on a single graph
            plt.figure(figsize=(10, 6))
            for i, intensities in enumerate(self.intensity):
                plt.plot(steps, intensities, label=f'Agent {i + 1}', color=f'C{i}')

            plt.title('Field Intensity Over Steps for All Agents')
            plt.xlabel('Steps')
            plt.ylabel('Field Intensity')
            plt.legend()

        plt.savefig(path)
        plt.close()
