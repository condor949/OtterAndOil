import numpy as np
from typing import Dict
from spaces import BaseSpace

from .BaseController import BaseController


class IntensityBasedController(BaseController):
    name = 'intensity'
    abbreviation = 'IBC'
    controller_type = 'individual'
    def __init__(self, vehicles, sim_time: int, sample_time: float, space: BaseSpace,
                 eps: float,
                 e_max_cap: float,
                 dynamic_error_max: bool,
                 smoothing: float,
                 FPS=30,
                 isolines=10,
                 mu=1):
        super().__init__(vehicles=vehicles,
                         sim_time=sim_time,
                         sample_time=sample_time,
                         space=space,
                         eps=eps,
                         e_max_cap=e_max_cap,
                         dynamic_error_max=dynamic_error_max,
                         smoothing=smoothing)
        self.m_f_prev = [space.get_intensity(vehicle.starting_point[1], vehicle.starting_point[0]) for vehicle in vehicles]
        self.mu = mu
        self.FPS = FPS
        self.isolines = isolines
        self.intensity = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.der = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.mu_tanh = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.sigmas = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.quality_array = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.type = 'Individual intensity based controller'
        self.times_outside = []
        self.u_controls = []
        self.dss = []
        self.n_rots = []
        self.n_forwards = []
        self.nus = []

    def __str__(self):
        return (f'---controller--------------------------------------------------------------------------\n'
                f'{self.type}\n'
                f'Sampling frequency: {round(1 / self.sample_time)} Hz\n'
                f'Sampling time: {self.sample_time} seconds\n'
                f'Simulation time: {round(self.sim_time)} seconds\n'
                f'Numbers of vehicles: {self.number_of_vehicles}')

    def matveev_law(self, vehicle, step, f_current, f_prev):
        self.der[vehicle, step] = (f_current - f_prev) / self.sample_time
        self.mu_tanh[vehicle, step] = self.mu * np.tanh(f_current) #- self.f_target)
        self.sigmas[vehicle, step] = -np.sign(self.der[vehicle, step] + self.mu_tanh[vehicle, step])
        #self.sigmas[vehicle, step] = -np.sign(self.der[vehicle, step] + ds * self.mu_tanh[vehicle, step])
        return self.sigmas[vehicle, step]

    def generate_control(self, positions, step, relative_velocities):
        m_f_current = [self.space.get_intensity(eta[1], eta[0]) for eta in positions]
        controls = []
        for i, vehicle in enumerate(self.vehicles):
            f_current = m_f_current[i]
            #print(f_current)
            f_prev = self.m_f_prev[i]
            nu = relative_velocities[i]
            self.nus.append(nu)
            sigma = self.matveev_law(i, step, f_current, f_prev)
            self.errors[i,step] = abs(f_current)
            e_norm = self.update_error_metrics(f_current)
            self.sum_error_values[i] += e_norm
            self.errors_max[i, step] = self.e_max
            self.errors_norm[i, step] = e_norm
            self.errors_avg[i] = self.moving_average(self.errors_norm[i], 1000)
            if sigma < 0:
                u_control = [vehicle.n_min, vehicle.n_max]
            elif sigma > 0:
                u_control = [vehicle.n_max, vehicle.n_min]
            else:
                u_control = [0, 0]
            controls.append(u_control)
            self.intensity[i, step] = f_current
            #self.quality_array[i, step] = self.space.get_nearest_contour_point_norm(positions[i][0], positions[i][1])

        self.m_f_prev = m_f_current
        return controls

    def track_snapshot(self) -> Dict[str, object]:
        if self.space is None:
            raise ValueError("Controller must be associated with a space before plotting tracks")

        colors = [self.colors[i] for i in range(self.number_of_vehicles)] if self.colors else None
        return {
            "grid_x": self.space.get_X(),
            "grid_y": self.space.get_Y(),
            "grid_z": self.space.get_Z(),
            "grid_size": self.space.grid_size,
            "target_isoline": self.space.target_isoline,
            "isolines": self.isolines,
            "fps": self.FPS,
            "colors": colors,
        }
