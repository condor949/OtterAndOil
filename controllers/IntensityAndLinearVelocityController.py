import numpy as np
from spaces import BaseSpace
from .IntensityBasedController import IntensityBasedController


class IntensityAndLinearVelocityController(IntensityBasedController):
    name = 'intensity_and_linear_velocity_nonlinear_control'
    def __init__(self, vehicles, sim_time: int, sample_time: float, space: BaseSpace,
                 eps: float,
                 e_max_cap: float,
                 dynamic_error_max: bool,
                 smoothing: float,
                 plot_config_path: str,
                 use_latex: bool = True,
                 FPS: int = 30,
                 isolines=10,
                 mu=1):
        super().__init__(vehicles=vehicles,
                         sim_time=sim_time,
                         sample_time=sample_time,
                         space=space,
                         FPS=FPS,
                         isolines=isolines,
                         mu=mu,
                         eps=eps,
                         e_max_cap=e_max_cap,
                         dynamic_error_max=dynamic_error_max,
                         smoothing=smoothing,
                         plot_config_path=plot_config_path,
                         use_latex=use_latex)
        self.type = 'Individual intensity controller with linear velocity nonlinear control'
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

    def matveev_law(self, vehicle, step, f_current, f_prev, ds):
        self.der[vehicle, step] = (f_current - f_prev) / self.sample_time
        self.mu_tanh[vehicle, step] = self.mu * np.tanh(f_current)
        self.sigmas[vehicle, step] = -np.sign(self.der[vehicle, step] + ds * self.mu_tanh[vehicle, step])
        return self.sigmas[vehicle, step]

    def generate_control(self, positions, step, relative_velocities):
        m_f_current = [self.space.get_intensity(eta[1], eta[0]) for eta in positions]
        controls = []
        for vehicle in self.vehicles:
            f_current = m_f_current[vehicle.serial_number]
            f_prev = self.m_f_prev[vehicle.serial_number]
            nu = relative_velocities[vehicle.serial_number]
            self.nus.append(nu)
            ds = np.sqrt(nu[0] ** 2 + nu[1] ** 2)
            self.dss.append(ds)
            sigma = self.matveev_law(vehicle.serial_number, step, f_current, f_prev, ds)
            n_forward = 100 - 80 * np.exp(-0.01 * f_current ** 2)#- self.f_target)**2)
            n_rot = 30 * sigma
            self.n_rots.append([n_rot, n_forward])
            if sigma < 0:
                u_control = [n_rot - n_forward, n_forward + n_rot]
            elif sigma > 0:
                u_control = [n_forward + n_rot, n_rot - n_forward]
            else:
                u_control = [0, 0]
            controls.append(u_control)
            self.u_controls.append(u_control)
            self.intensity[vehicle.serial_number, step] = f_current
            self.quality_array[vehicle.serial_number, step] = self.space.get_nearest_contour_point_norm(positions[vehicle.serial_number][0], positions[vehicle.serial_number][1])

        self.m_f_prev = m_f_current
        return controls
