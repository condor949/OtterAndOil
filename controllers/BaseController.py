from abc import ABC
from collections.abc import Sequence
from typing import Dict

import numpy as np

from spaces import BaseSpace

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

    def get_plot_path(self, plot_name: str, extension: str) -> str:
        if self.data_storage is None:
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
                raise AttributeError(f"Controller '{self.name}' has no attribute '{name}' for plotting")
            snapshot[name] = getattr(self, name)
        return snapshot

    def track_snapshot(self) -> Dict[str, object]:
        """Return metadata required to render track plots for the controller."""
        raise NotImplementedError("Controller must implement track_snapshot to support track plotting")
