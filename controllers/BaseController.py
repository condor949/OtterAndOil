import logging
from abc import ABC
from collections.abc import Sequence
from typing import Dict, List, Optional

import numpy as np

from spaces import BaseSpace
from tqdm import tqdm


logger = logging.getLogger(__name__)

class BaseController(ABC):
    name = 'base_controller'
    abbreviation = 'BASE'
    controller_type = None
    def __init__(self,
                 vehicles,
                 sim_time: int,
                 sample_time: float,
                 space: BaseSpace,
                 eps: float,
                 e_max_cap: float,
                 dynamic_error_max: bool,
                 smoothing: float):
        self.sample_time = sample_time
        self.sim_time = sim_time
        self.N = round(sim_time / sample_time) + 1
        self.simTime = np.arange(start=0, stop=self.sample_time*self.N, step=sample_time)#[:, None]
        self.space = space
        self.number_of_vehicles = len(vehicles)
        self.vehicles = vehicles
        self.colors = {i: vehicle.color for i, vehicle in enumerate(vehicles)}
        self.data_storage = None
        self.sim_data: Optional[List[np.ndarray]] = None
        self.sum_error_values = np.zeros(self.number_of_vehicles, dtype=float)
        self.eps = eps
        self.e_max = self.eps
        self.smoothing = smoothing
        self.e_max_cap = e_max_cap
        self.dynamic_error_max = dynamic_error_max
        self.errors = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.errors_norm = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.errors_max = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.errors_avg = np.zeros((self.number_of_vehicles, self.N), dtype=float)

    def generate_control(self, positions, step, relative_velocities) -> Sequence:
        pass

    def set_data_storage(self, data_storage) -> None:
        self.data_storage = data_storage
        logger.debug("Data storage configured for controller '%s' -> %s", self.name, data_storage)

    def get_plot_path(self, plot_name: str, extension: str) -> str:
        if self.data_storage is None:
            logger.error("Attempted to access plot path '%s.%s' before configuring data storage", plot_name, extension)
            raise RuntimeError("Data storage must be configured before generating plot paths")
        return self.data_storage.get_path(plot_name, extension)

    def get_average_error(self):
        return self.sum_error_values/self.sim_time

    def update_error_metrics(self, f_current):
        e = abs(f_current)
        if self.dynamic_error_max:
            # Обновление с экспоненциальным сглаживанием
            self.e_max = max(self.smoothing * self.e_max, e)

            # Ограничение сверху
            self.e_max = min(self.e_max, self.e_max_cap)
        else:
            self.e_max = self.e_max_cap
        # Нормализация
        e_norm = e / self.e_max if self.e_max > 0 else 0.0

        return e_norm

    def moving_average(self, a, n=20):
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return np.concatenate([
            a[:n - 1],  # первые значения — без усреднения
            ret[n - 1:] / n
        ])

    def snapshot(self, *variable_names: str) -> Dict[str, object]:
        requested = {name for name in variable_names if name}
        snapshot: Dict[str, object] = {}
        for name in requested:
            if not hasattr(self, name):
                logger.error("Snapshot requested unknown attribute '%s' on controller '%s'", name, self.name)
                raise AttributeError(f"Controller '{self.name}' has no attribute '{name}' for plotting")
            snapshot[name] = getattr(self, name)
        return snapshot

    def track_snapshot(self) -> Dict[str, object]:
        """Return metadata required to render track plots for the controller."""
        raise NotImplementedError("Controller must implement track_snapshot to support track plotting")

    def simultaneous_simulate(self) -> List[np.ndarray]:
        # Detect DOF from the first vehicle's nu vector length
        # 3-DOF: [u, v, r] -> DOF=3
        # 6-DOF: [u, v, w, p, q, r] -> DOF=6
        DOF = len(self.vehicles[0].nu) if self.vehicles else 6

        m_nu = []
        m_u_actual = []
        m_eta = []

        logger.info(
            "Controller '%s' starting simultaneous simulation with %d vehicle(s) and %d steps (DOF=%d)",
            self.name,
            self.number_of_vehicles,
            self.N,
            DOF,
        )

        for vehicle in self.vehicles:
            # Initialize eta based on DOF
            # For 3-DOF: [y, x, psi] (matching 6-DOF convention where eta[0]=y, eta[1]=x)
            # For 6-DOF: [y, x, z, phi, theta, psi]
            if DOF == 3:
                # 3-DOF: starting_point is [x, y], but we store as [y, x, psi] to match convention
                m_eta.append(np.array([vehicle.starting_point[1], vehicle.starting_point[0], 0.0], float))
            else:
                # 6-DOF: starting_point is [x, y], stored as [y, x, z, phi, theta, psi]
                m_eta.append(np.array([vehicle.starting_point[1], vehicle.starting_point[0], 0, 0, 0, 0], float))
            m_nu.append(vehicle.nu)
            m_u_actual.append(vehicle.u_actual)

        sim_data = [np.empty([self.N, 2 * DOF + 2 * vehicle.dimU], float) for vehicle in self.vehicles]

        for step in tqdm(range(0, self.N), desc=f"Vehicle Simulation x{self.number_of_vehicles}"):
            m_u_control = self.generate_control(m_eta, step, m_nu)

            for internal_number, vehicle in enumerate(self.vehicles):
                eta = m_eta[internal_number]
                nu = m_nu[internal_number]
                u_actual = m_u_actual[internal_number]
                u_control = m_u_control[internal_number]

                signals = np.hstack((eta, nu, u_control, u_actual))
                sim_data[internal_number][step, :] = signals

                nu, u_actual = vehicle.dynamics(eta, nu, u_actual, u_control, self.sample_time)
                eta = vehicle.repositioning(eta, nu, self.sample_time)

                m_eta[internal_number] = eta
                m_nu[internal_number] = nu
                m_u_actual[internal_number] = u_actual

        self.sim_data = sim_data
        logger.info("Controller '%s' completed simulation", self.name)
        return sim_data
