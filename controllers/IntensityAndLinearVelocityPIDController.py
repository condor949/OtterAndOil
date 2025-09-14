import numpy as np
from spaces import BaseSpace
from .IntensityBasedController import IntensityBasedController


class IntensityAndLinearVelocityPIDController(IntensityBasedController):
    name = 'intensity_and_linear_velocity_nonlinear_pid_control'
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
        self.type = 'Individual intensity controller with linear velocity PID control'
        self.v0 = 20
        self.t_max = 5
        self.k_I = 30
        self.k_P = 15
        self.time_outside = 0
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
            #print(f_current)
            f_prev = self.m_f_prev[vehicle.serial_number]
            nu = relative_velocities[vehicle.serial_number]
            self.nus.append(nu)
            ds = np.sqrt(nu[0] ** 2 + nu[1] ** 2)
            self.dss.append(ds)
            sigma = self.matveev_law(vehicle.serial_number, step, f_current, f_prev, ds)
            if abs(f_current) < self.eps:
                self.time_outside = 0
            else:
                self.time_outside += self.sample_time  # интегрирование
            self.times_outside.append(self.time_outside)
            e_norm = self.update_error_metrics(f_current)
            self.sum_error_values[vehicle.serial_number] += e_norm
            self.errors_max[vehicle.serial_number, step] = self.e_max
            self.errors_norm[vehicle.serial_number, step] = e_norm
            time_norm = min(self.time_outside / self.t_max, 1.0)
            n_forward = self.v0 + self.k_P * e_norm + self.k_I * self.time_outside
            n_rot = 30 * sigma
            self.n_rots.append([n_rot, n_forward])
            n_min = np.clip(n_rot - n_forward, vehicle.n_min, vehicle.n_max)
            n_max = np.clip(n_rot + n_forward, vehicle.n_min, vehicle.n_max)
            if sigma < 0:
                u_control = [n_min, n_max]
            elif sigma > 0:
                u_control = [n_max, n_min]
            else:
                u_control = [0, 0]
            controls.append(u_control)
            self.u_controls.append(u_control)
            self.intensity[vehicle.serial_number, step] = f_current
            self.quality_array[vehicle.serial_number, step] = self.space.get_nearest_contour_point_norm(positions[vehicle.serial_number][0], positions[vehicle.serial_number][1])

        self.m_f_prev = m_f_current
        return controls
