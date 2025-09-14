import os
import json
import numpy as np
import matplotlib.pyplot as plt
from abc import ABC

from spaces import BaseSpace
from collections.abc import Sequence


class BaseController(ABC):
    name = 'base_controller'
    def __init__(self,
                 vehicles,
                 sim_time: int,
                 sample_time: float,
                 space: BaseSpace,
                 eps: float,
                 e_max_cap: float,
                 dynamic_error_max: bool,
                 smoothing: float,
                 plot_config_path: str,
                 use_latex: bool = True):
        self.sample_time = sample_time
        self.sim_time = sim_time
        self.N = round(sim_time / sample_time) + 1
        self.simTime = np.arange(start=0, stop=self.sample_time*self.N, step=sample_time)#[:, None]
        self.space = space
        self.number_of_vehicles = len(vehicles)
        self.vehicles = vehicles
        self.colors = {vehicle.serial_number: vehicle.color for vehicle in vehicles}
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
        self.dpi = 150
        self.use_latex = use_latex

        with open(plot_config_path, 'r') as f:
            self.plot_config = json.load(f)

        for plot_name, config in self.plot_config.items():
            method = self._create_plotting_method(plot_name, config)
            setattr(self, f'plotting_{plot_name}', method)
            save_method = self._create_store_method(plot_name, config)
            setattr(self, f'store_{plot_name}', save_method)

    def generate_control(self, positions, step, relative_velocities) -> Sequence:
        pass

    def set_data_storage(self, data_storage) -> None:
        self.data_storage = data_storage

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

    def _resolve_xy(self, config, x=None, y=None):
        print(np.array(x).shape)
        print(np.array(y).shape)
        if y is None:
            var_y = config.get("y")
            if var_y and hasattr(self, var_y):
                y = getattr(self, var_y)
            else:
                raise ValueError(f"y data must be provided or '{var_y}' must exist as an attribute")

        if not isinstance(y, np.ndarray):
            raise TypeError(f"y (from '{config.get('y', 'unknown')}') must be a NumPy array, got {type(y)}")

        if x is None:
            var_x = config.get("x")
            if var_x and hasattr(self, var_x):
                x = getattr(self, var_x)
            else:
                x = np.arange(y.shape[1]) if y.ndim == 2 else np.arange(len(y))

        if not isinstance(x, np.ndarray):
            try:
                x = np.asarray(x)
            except Exception as e:
                raise TypeError(f"x (from '{config.get('x', 'unknown')}') could not be converted to NumPy array: {e}")

        if y.ndim == 2:
            if x.ndim > 1:
                if x.size == y.shape[1]:
                    x = x.reshape(-1)  # приведение к совместимому виду
                else:
                    raise ValueError(f"x has shape {x.shape}, incompatible with y shape {y.shape}")
            elif x.shape[0] != y.shape[1]:
                raise ValueError(f"x has length {x.shape[0]}, but expected {y.shape[1]} to match y")

        return x, y

    def _create_plotting_method(self, name, config):
        def plotting_method(y=None, x=None, separate_plots=False, store_plot=False, big_picture=False,
                            for_publication=False, colors=False, **kwargs):
            x, y = self._resolve_xy(config, x, y)

            if y.ndim == 1:
                y = np.expand_dims(y, axis=0)

            plt.rc('text', usetex=self.use_latex)
            plt.rc('font', size=25 if for_publication else 12)

            fig_size_big = (30, 20)
            fig_size_small = (10, 6)
            fig_size = fig_size_big if big_picture else fig_size_small

            if separate_plots:
                fig, axes = plt.subplots(y.shape[0], 1, figsize=(fig_size[0], fig_size[1] * y.shape[0]), sharex=True,
                                         dpi=self.dpi)
                if y.shape[0] == 1:
                    axes = [axes]
                for i, (data, ax) in enumerate(zip(y, axes)):
                    if colors:
                        ax.plot(x, data, label=config.get("legend", "Series {i}").format(i=i + 1), color=colors[i])
                    else:
                        ax.plot(x, data, label=config.get("legend", "Series {i}").format(i=i + 1))
                    ax.set_xlabel(config.get("x_label", "X"))
                    ax.set_ylabel(config.get("y_label", "Y"))
                    ax.legend()
                    if not store_plot:
                        ax.set_title(config.get("title", f"Plot {name}") + f" — Series {i + 1}")
            else:
                plt.figure(figsize=fig_size, dpi=self.dpi)
                for i, data in enumerate(y):
                    if colors:
                        plt.plot(x, data, label=config.get("legend", "Series {i}").format(i=i + 1), color=colors[i])
                    else:
                        plt.plot(x, data, label=config.get("legend", "Series {i}").format(i=i + 1))
                plt.xlabel(config.get("x_label", "X"))
                plt.ylabel(config.get("y_label", "Y"))
                plt.legend()
                if not store_plot:
                    plt.title(config.get("title", f"Plot {name}"))

            plt.tight_layout()
            if store_plot:
                plt.savefig(self.data_storage.get_path(f"{name}","png"))
                plt.close()
            else:
                plt.show()
                plt.close()

        return plotting_method

    def _create_store_method(self, name, config):
        def store_method(y=None, x=None):
            x, y = self._resolve_xy(config, x, y)

            if y.ndim == 1:
                y = np.expand_dims(y, axis=0)

            data_dict = {
                "x": x,
                "y": y,
                "x_label": config.get("x_label", "X"),
                "y_label": config.get("y_label", "Y")
            }
            path = os.path.join("plot", f"{name}.npy")
            np.save(path, data_dict)
            print(f"Saved data to {path}")

        return store_method
