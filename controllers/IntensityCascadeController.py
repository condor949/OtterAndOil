import numpy as np
from typing import Dict
from spaces import BaseSpace

from lib.gnc import sat
from .IntensityAndLinearVelocityPIDController import IntensityAndLinearVelocityPIDController


class IntensityCascadeController(IntensityAndLinearVelocityPIDController):
    """
    Cascade: outer (Matveev + PI on intensity) -> r_d, tau_X_cmd;
    inner (PI on yaw) -> tau_N; allocation -> [n1, n2].
    """
    name = 'intensity_cascade'
    abbreviation = 'ICC'
    controller_type = 'individual'

    def __init__(self, vehicles, sim_time: int, sample_time: float, space: BaseSpace,
                 eps: float,
                 e_max_cap: float,
                 dynamic_error_max: bool,
                 smoothing: float,
                 FPS: int = 30,
                 isolines=10,
                 mu=1,
                 r0=0.3,
                 r_d_max=0.5,
                 omega_minus=None,
                 omega_plus=None,
                 tau_X_min=0.0,
                 tau_X_max=250.0,
                 tau_N_min=-50.0,
                 tau_N_max=50.0,
                 kpr=80.0,
                 kir=20.0,
                 k_tau_X=1.0,
                 k_tau_N=1.0,
                 X_MAX=20.0,
                 N_MAX=10.0,
                 log_every_n_steps=50):
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
                         smoothing=smoothing)
        self.type = 'Intensity cascade controller (Matveev -> r_d/tau_X -> PI yaw -> allocation)'
        self.r0 = r0
        self.r_d_max = r_d_max
        # Matveev: omega(t) = 0.5*[(1-sigma)*omega_minus + (1+sigma)*omega_plus]
        self.omega_minus = omega_minus if omega_minus is not None else -r0
        self.omega_plus = omega_plus if omega_plus is not None else r0
        self.tau_X_min = tau_X_min
        self.tau_X_max = tau_X_max
        self.tau_N_min = tau_N_min
        self.tau_N_max = tau_N_max
        self.kpr = kpr
        self.kir = kir
        self.k_tau_X = k_tau_X
        self.k_tau_N = k_tau_N
        self.X_MAX = X_MAX
        self.N_MAX = N_MAX
        self.log_every_n_steps = log_every_n_steps
        # Per-vehicle integral for yaw PI (anti-windup via conditional integration)
        self.int_e_r = [0.0] * self.number_of_vehicles
        # Logging (same style as base: lists appended per step per vehicle)
        self.log_sigma = []
        self.log_r_d = []
        self.log_r = []
        self.log_tau_X = []
        self.log_tau_N = []
        self.log_n1 = []
        self.log_n2 = []
        self.log_f_current = []
        self.log_e_norm = []
        self.log_time_outside = []

    def __str__(self):
        return (f'---controller--------------------------------------------------------------------------\n'
                f'{self.type}\n'
                f'Sampling frequency: {round(1 / self.sample_time)} Hz\n'
                f'Sampling time: {self.sample_time} seconds\n'
                f'Simulation time: {round(self.sim_time)} seconds\n'
                f'Numbers of vehicles: {self.number_of_vehicles}')

    def generate_control(self, positions, step, relative_velocities):
        m_f_current = [self.space.get_intensity(eta[1], eta[0]) for eta in positions]
        controls = []
        for i, vehicle in enumerate(self.vehicles):
            f_current = m_f_current[i]
            f_prev = self.m_f_prev[i]
            nu = relative_velocities[i]
            self.nus.append(nu)
            ds = np.sqrt(nu[0] ** 2 + nu[1] ** 2)
            self.dss.append(ds)
            sigma = self.matveev_law(i, step, f_current, f_prev, ds)

            if abs(f_current) < self.eps:
                self.time_outside = 0
            else:
                self.time_outside += self.sample_time
            self.times_outside.append(self.time_outside)
            e_norm = self.update_error_metrics(f_current)
            self.sum_error_values[i] += e_norm
            self.errors_max[i, step] = self.e_max
            self.errors_norm[i, step] = e_norm

            # ----- Outer loop: 3-DOF commands -----
            # Matveev desired yaw rate: omega(t) = 0.5*[(1-sigma)*omega_minus + (1+sigma)*omega_plus]
            r_d_raw = 0.5 * ((1.0 - sigma) * self.omega_minus + (1.0 + sigma) * self.omega_plus)
            r_d = sat(r_d_raw, -self.r_d_max, self.r_d_max)

            # Surge command: same PI on intensity as tau_X_cmd (Option A)
            tau_X_cmd = self.v0 + self.k_P * e_norm + self.k_I * self.time_outside
            tau_X = sat(tau_X_cmd, self.tau_X_min, self.tau_X_max)

            # ----- Inner loop: yaw PI -> tau_N -----
            r = nu[2] if len(nu) == 3 else nu[5]
            e_r = r_d - r
            self.int_e_r[i] += self.sample_time * e_r
            tau_N_raw = self.kpr * e_r + self.kir * self.int_e_r[i]
            tau_N = sat(tau_N_raw, self.tau_N_min, self.tau_N_max)
            # Anti-windup: conditional integration (only integrate when not saturated)
            if tau_N != tau_N_raw:
                self.int_e_r[i] = (tau_N - self.kpr * e_r) / self.kir if self.kir != 0 else self.int_e_r[i]

            # ----- Scaling then saturation before allocation -----
            tau_X_scaled = self.k_tau_X * tau_X
            tau_N_scaled = self.k_tau_N * tau_N
            tau_X = float(np.clip(tau_X_scaled, -self.X_MAX, self.X_MAX))
            tau_N = float(np.clip(tau_N_scaled, -self.N_MAX, self.N_MAX))

            # ----- Allocation -----
            n1, n2 = vehicle.controlAllocation(tau_X, tau_N)
            u_control = np.array([n1, n2], float)
            controls.append(u_control)
            self.u_controls.append(u_control)
            self.n_rots.append([r_d, tau_X])
            self.n_forwards.append(tau_X_cmd)
            self.intensity[i, step] = f_current

            # Logging
            self.log_sigma.append(float(sigma))
            self.log_r_d.append(float(r_d))
            self.log_r.append(float(r))
            self.log_tau_X.append(float(tau_X))
            self.log_tau_N.append(float(tau_N))
            self.log_n1.append(float(n1))
            self.log_n2.append(float(n2))
            self.log_f_current.append(float(f_current))
            self.log_e_norm.append(float(e_norm))
            self.log_time_outside.append(float(self.time_outside))

            # Periodic log every N steps
            if self.log_every_n_steps and step % self.log_every_n_steps == 0:
                t = step * self.sample_time
                u_actual = getattr(vehicle, 'u_actual', np.array([0.0, 0.0]))
                u_act = u_actual if isinstance(u_actual, np.ndarray) else np.array(u_actual, float)
                print(f"[ICC step {step}] t={t:.2f} f={f_current:.4f} sigma={sigma} r_d={r_d:.4f} r={r:.4f} "
                      f"tau_X={tau_X:.2f} tau_N={tau_N:.2f} n1={n1:.2f} n2={n2:.2f} u_actual=[{u_act[0]:.2f},{u_act[1]:.2f}]")

        self.m_f_prev = m_f_current
        return controls

    def track_snapshot(self) -> Dict[str, object]:
        if self.space is None:
            raise ValueError("Controller must be associated with a space before plotting tracks")
        colors = [self.colors[i] for i in range(self.number_of_vehicles)] if self.colors else None
        serial_numbers = [vehicle.serial_number for vehicle in self.vehicles]
        return {
            "grid_x": self.space.get_X(),
            "grid_y": self.space.get_Y(),
            "grid_z": self.space.get_Z(),
            "grid_size": self.space.grid_size,
            "target_isoline": self.space.target_isoline,
            "isolines": self.isolines,
            "fps": self.FPS,
            "colors": colors,
            "serial_numbers": serial_numbers,
        }
