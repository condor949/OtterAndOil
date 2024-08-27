from .BaseController import BaseController
import numpy as np


def oil_intensity(x, y):
    return -(x - 10) ** 2 - (y - 10) ** 2 + 30


class IntensityBasedController(BaseController):

    def __init__(self, starting_points, sample_time, f0=0, mu=1):
        super().__init__()
        self.m_f_prev = [oil_intensity(point[0], point[1]) for point in starting_points]
        self.sample_time = sample_time
        self.f0 = f0
        self.mu = mu

    def ivan_law(self, f_current, f_prev):
        sigma = -np.sign((f_current - f_prev) / self.sample_time + self.mu * np.tanh(f_current - self.f0))
        return sigma

    def generate_control(self, vehicles, positions):
        m_f_current = [oil_intensity(eta[0], eta[1]) for eta in positions]

        controls = []
        for k in range(len(vehicles)):

            f_current = m_f_current[k]
            f_prev = self.m_f_prev[k]

            sigma = self.ivan_law(f_current, f_prev)

            if sigma < 0:
                u_control = [vehicles[k].n_min, vehicles[k].n_max]
            elif sigma > 0:
                u_control = [vehicles[k].n_max, vehicles[k].n_min]
            else:
                u_control = [0, 0]
            controls.append(u_control)
        self.m_f_prev = m_f_current
        return controls