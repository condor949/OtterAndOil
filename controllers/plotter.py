import json
from collections.abc import Iterable as IterableCollection
from dataclasses import dataclass
from functools import partial
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rc, rcParams, rcParamsDefault
from tqdm import tqdm

from tools.dataStorage import DataStorage
from tools.random_generators import color_generator

from .BaseController import BaseController

rcParams.update(rcParamsDefault)


def cm2inch(value):  # inch to cm
    return value / 2.54


figSize1 = [25, 25]  # figure1 size in cm
bigFigSize1 = [50, 26]  # larger figure1 size in cm


@dataclass
class PlotRenderOptions:
    big_picture: bool = False
    separate_plots: bool = False
    store_plots: bool = False
    for_publication: bool = False
    use_colors: bool = True


@dataclass
class TrackRenderOptions:
    big_picture: bool = False
    not_animated: bool = False
    store_plot: bool = False


@dataclass
class PlotSeries:
    controller: BaseController
    x: np.ndarray
    y: np.ndarray
    label: str
    color: Optional[str]


class ControllerPlotter:
    """Render plots for one or multiple controllers based on a configuration file."""

    def __init__(self,
                 controllers: Sequence[BaseController],
                 plot_config_path: str,
                 use_latex: bool = True,
                 dpi: int = 150,
                 plot_options: Optional[PlotRenderOptions] = None,
                 track_options: Optional[TrackRenderOptions] = None,
                 data_storage: Optional[DataStorage] = None) -> None:
        self._controllers: List[BaseController] = list(controllers)
        self._plot_config_path = plot_config_path
        self._use_latex = use_latex
        self._dpi = dpi
        self._plot_options = plot_options or PlotRenderOptions()
        self._track_options = track_options or TrackRenderOptions()
        self._storage = data_storage.create_child_storage("plots") if data_storage else None
        self._subset_storage: Dict[Tuple[str, ...], DataStorage] = {}
        with open(plot_config_path, 'r', encoding='utf-8') as config_file:
            self._plot_config = json.load(config_file)

        for plot_name, config in self._plot_config.items():
            plot_method = self._create_plotting_method(plot_name, config)
            setattr(self, f'plotting_{plot_name}', plot_method)
            store_method = self._create_store_method(plot_name, config)
            setattr(self, f'store_{plot_name}', store_method)

    @property
    def controllers(self) -> List[BaseController]:
        return list(self._controllers)

    def update_controllers(self, controllers: Iterable[BaseController]) -> None:
        self._controllers = list(controllers)

    def plotting_track(self,
                       controller: BaseController,
                       swarmData=None) -> None:
        store_plot = self._track_options.store_plot
        animate = store_plot and not self._track_options.not_animated
        if swarmData is None:
            raise ValueError("Simulation data must be provided for track plotting")

        track_snapshot = controller.track_snapshot()
        if not isinstance(track_snapshot, dict):
            raise TypeError("track_snapshot must return a mapping with track metadata")

        required_keys = [
            "grid_x",
            "grid_y",
            "grid_z",
            "grid_size",
            "target_isoline",
            "isolines",
            "fps",
        ]
        missing_keys = [key for key in required_keys if key not in track_snapshot]
        if missing_keys:
            raise KeyError(
                "track_snapshot is missing required keys: " + ", ".join(missing_keys)
            )

        grid_x = np.asarray(track_snapshot["grid_x"])
        grid_y = np.asarray(track_snapshot["grid_y"])
        grid_z = np.asarray(track_snapshot["grid_z"])
        grid_size = int(track_snapshot["grid_size"])
        target_isoline = track_snapshot["target_isoline"]
        isolines = track_snapshot["isolines"]
        fps = int(track_snapshot["fps"])
        colors_metadata = track_snapshot.get("colors")
        provided_colors = list(colors_metadata) if colors_metadata is not None else []

        if self._track_options.big_picture:
            fig = plt.figure(figsize=(cm2inch(bigFigSize1[0]), cm2inch(bigFigSize1[1])),
                             dpi=self._dpi)
        else:
            fig = plt.figure(figsize=(cm2inch(figSize1[0]), cm2inch(figSize1[1])),
                             dpi=self._dpi)

        plt.rc('text', usetex=self._use_latex)
        rc('font', size=30)
        contour = plt.contour(grid_x,
                              grid_y,
                              grid_z,
                              levels=isolines,
                              cmap='viridis')

        plt.clabel(contour, inline=True)
        plt.xlabel('X, m / East')
        plt.ylabel('Y, m / North')
        plt.contour(grid_x,
                    grid_y,
                    grid_z,
                    levels=[target_isoline],
                    colors='red')

        plotData = {}
        quivers = []

        def anim_function(num, plotData, quivers):
            for i, (line, dataSet) in enumerate(plotData.items()):
                line.set_data(dataSet[0:2, :num])

                if num < dataSet.shape[1] - 1:
                    dx = dataSet[0, num + 1] - dataSet[0, num]
                    dy = dataSet[1, num + 1] - dataSet[1, num]
                else:
                    dx = dataSet[0, num] - dataSet[0, num - 1]
                    dy = dataSet[1, num] - dataSet[1, num - 1]

                norm = np.hypot(dx, dy)
                if norm < 1e-6:
                    dx, dy = 1.0, 0.0
                else:
                    dx, dy = dx / norm, dy / norm
                    arrow_length = 2.0
                    dx *= arrow_length
                    dy *= arrow_length

                quivers[i].set_offsets([dataSet[0, num], dataSet[1, num]])
                quivers[i].set_UVC(dx, dy)

            return list(plotData.keys()) + quivers

        color_gen = color_generator()
        for i, simData in enumerate(swarmData):
            x = simData[:, 0]
            y = simData[:, 1]
            z = simData[:, 2]

            if grid_size <= 0:
                raise ValueError("Track snapshot must define a positive grid size")

            step = max(len(x) // grid_size, 1)
            N = y[::step]
            E = x[::step]
            D = z[::step]

            dataSet = np.array([N, E, -D])

            if i < len(provided_colors):
                color = provided_colors[i]
            else:
                color = next(color_gen)
            start_x = dataSet[0][0]
            start_y = dataSet[1][0]
            initial_dx = dataSet[0][1] - dataSet[0][0]
            initial_dy = dataSet[1][1] - dataSet[1][0]
            plt.plot(start_x, start_y, marker='*', markersize=10, color=color, label='_nolegend_',
                     zorder=10)

            quiv = plt.quiver(
                start_x, start_y, initial_dx, initial_dy,
                angles='xy',
                scale_units='xy',
                scale=1,
                color=color,
                width=0.02, zorder=15, label='_nolegend_'
            )
            quivers.append(quiv)

            line = plt.plot(dataSet[0], dataSet[1], lw=2, c=color, zorder=10, label=f'agent {i + 1}')[0]
            plotData[line] = dataSet

        plt.legend()

        if store_plot:
            plt.tight_layout()
            plt.savefig(controller.get_plot_path('track', "png"))
        else:
            plt.tight_layout()
            plt.title('Track in the intensity field')
            plt.show()

        if animate:
            ani = animation.FuncAnimation(fig,
                                          partial(anim_function, plotData=plotData, quivers=quivers),
                                          frames=grid_size,
                                          interval=200,
                                          blit=False,
                                          repeat=True)

            update_func = lambda _i, _n: progress_bar.update(1)
            with tqdm(total=getattr(ani, "_save_count"), desc="Animation Writing") as progress_bar:
                try:
                    writer = animation.FFMpegWriter(fps=fps)
                except Exception:
                    writer = animation.PillowWriter(fps=fps)

                ani.save(controller.get_plot_path('track', "gif"),
                         writer=writer,
                         progress_callback=update_func)

        plt.close()

    def _create_plotting_method(self, name: str, config: dict):
        def plotting_method(*,
                            controllers: Optional[Sequence[BaseController]] = None,
                            x: Optional[np.ndarray] = None,
                            y: Optional[np.ndarray] = None,
                            combine: bool = False) -> None:
            store_plot = self._plot_options.store_plots
            use_colors = self._plot_options.use_colors
            target_controllers = list(controllers) if controllers is not None else self._controllers
            if not target_controllers:
                raise ValueError("No controllers are available for plotting")

            if combine:
                series = self._collect_all_series(target_controllers,
                                                  config,
                                                  x_override=x,
                                                  y_override=y,
                                                  use_colors=use_colors)
                combined_path = None
                if store_plot:
                    combined_path = self._resolve_combined_plot_path(name, target_controllers)
                self._render_series(name,
                                    series,
                                    config,
                                    combined=True,
                                    store_plot=store_plot,
                                    output_path=combined_path)
            else:
                for controller in target_controllers:
                    series = self._collect_series(controller,
                                                  config,
                                                  x_override=x,
                                                  y_override=y,
                                                  use_colors=use_colors)
                    self._render_series(name,
                                        series,
                                        config,
                                        combined=False,
                                        target_controller=controller,
                                        store_plot=store_plot)

        return plotting_method

    def _create_store_method(self, name: str, config: dict):
        def store_method(*,
                         controllers: Optional[Sequence[BaseController]] = None,
                         x: Optional[np.ndarray] = None,
                         y: Optional[np.ndarray] = None) -> None:
            target_controllers = list(controllers) if controllers is not None else self._controllers
            if not target_controllers:
                raise ValueError("No controllers are available for storing plot data")

            for controller in target_controllers:
                series = self._collect_series(controller,
                                              config,
                                              x_override=x,
                                              y_override=y,
                                              use_colors=False)
                if not series:
                    continue
                data_x = series[0].x
                data_y = np.vstack([item.y for item in series])
                data_dict = {
                    "x": data_x,
                    "y": data_y,
                    "x_label": config.get("x_label", "X"),
                    "y_label": config.get("y_label", "Y")
                }
                np.save(controller.get_plot_path(name, 'npy'), data_dict)

        return store_method

    def _collect_all_series(self,
                            controllers: Sequence[BaseController],
                            config: dict,
                            x_override: Optional[np.ndarray],
                            y_override: Optional[np.ndarray],
                            use_colors: bool) -> List[PlotSeries]:
        series: List[PlotSeries] = []
        index = 1
        for controller in controllers:
            controller_series = self._collect_series(controller,
                                                     config,
                                                     x_override=x_override,
                                                     y_override=y_override,
                                                     use_colors=use_colors,
                                                     start_index=index)
            series.extend(controller_series)
            index += len(controller_series)
        return series

    def _collect_series(self,
                        controller: BaseController,
                        config: dict,
                        x_override: Optional[np.ndarray],
                        y_override: Optional[np.ndarray],
                        use_colors: bool,
                        start_index: int = 1) -> List[PlotSeries]:
        x_key = config.get("x")
        y_key = config.get("y")
        requested = []
        if x_override is None and x_key:
            requested.append(x_key)
        if y_override is None and y_key:
            requested.append(y_key)
        if use_colors:
            requested.append("colors")
        snapshot = controller.snapshot(*requested)

        x_data = x_override if x_override is not None else self._to_array(snapshot.get(x_key), "x")
        y_source = y_override if y_override is not None else snapshot.get(y_key)
        y_array = self._to_array(y_source, "y")
        if y_array.ndim == 1:
            y_array = np.expand_dims(y_array, axis=0)

        colors = None
        if use_colors:
            colors = self._normalize_colors(snapshot.get("colors"), y_array.shape[0])

        legend_template = config.get("legend", "Series {i}")
        series: List[PlotSeries] = []
        for offset, row in enumerate(y_array):
            label = legend_template.format(i=start_index + offset,
                                           controller=controller.abbreviation,
                                           controller_name=controller.name,
                                           vehicle=offset + 1)
            color = colors[offset] if colors and offset < len(colors) else None
            series.append(PlotSeries(controller=controller,
                                     x=x_data,
                                     y=row,
                                     label=label,
                                     color=color))
        return series

    @staticmethod
    def _to_array(value, axis: str) -> np.ndarray:
        if value is None:
            raise ValueError(f"{axis} data must be provided by the controller snapshot or via override")
        array = np.asarray(value)
        if axis == "x" and array.ndim > 1:
            if array.size == array.shape[-1]:
                array = array.reshape(-1)
            else:
                raise ValueError(f"x data with shape {array.shape} cannot be reshaped into 1-D array")
        return array

    @staticmethod
    def _normalize_colors(colors, expected: int) -> Optional[List[str]]:
        if colors is None:
            return None
        if isinstance(colors, dict):
            ordered = [color for _, color in sorted(colors.items())]
            return ordered[:expected]
        try:
            sequence = list(colors)
        except TypeError as exc:  # pragma: no cover - defensive programming
            raise TypeError("Colors must be a sequence or mapping") from exc
        if len(sequence) < expected:
            sequence.extend([None] * (expected - len(sequence)))
        return sequence

    def _render_series(self,
                       name: str,
                       series: Sequence[PlotSeries],
                       config: dict,
                       *,
                       combined: bool,
                       target_controller: Optional[BaseController] = None,
                       store_plot: bool = False,
                       output_path: Optional[str] = None) -> None:
        if not series:
            return

        plt.rc('text', usetex=self._use_latex)
        plt.rc('font', size=25 if self._plot_options.for_publication else 12)

        fig_size = (30, 20) if self._plot_options.big_picture else (10, 6)
        title = config.get("title", f"Plot {name}")
        if target_controller is not None and not combined:
            title = f"{title} — {target_controller.name}"

        if self._plot_options.separate_plots:
            fig, axes = plt.subplots(len(series), 1,
                                     figsize=(fig_size[0], fig_size[1] * len(series)),
                                     sharex=True,
                                     dpi=self._dpi)
            axes_list = list(axes) if isinstance(axes, IterableCollection) else [axes]
            for ax, item in zip(axes_list, series):
                color = item.color
                ax.plot(item.x, item.y, label=item.label, color=color if color else None)
                ax.set_xlabel(config.get("x_label", "X"))
                ax.set_ylabel(config.get("y_label", "Y"))
                ax.legend()
                if not store_plot:
                    ax.set_title(title)
        else:
            plt.figure(figsize=fig_size, dpi=self._dpi)
            for item in series:
                color = item.color
                plt.plot(item.x, item.y, label=item.label, color=color if color else None)
            plt.xlabel(config.get("x_label", "X"))
            plt.ylabel(config.get("y_label", "Y"))
            plt.legend()
            if not store_plot:
                plt.title(title)

        plt.tight_layout()
        if store_plot:
            path = output_path
            if path is None and target_controller is not None:
                path = target_controller.get_plot_path(name, 'png')
            if path is None:
                raise RuntimeError("Unable to resolve storage path for the plot")
            plt.savefig(path)
            plt.close()
        else:
            plt.show()
            plt.close()

    def _resolve_combined_plot_path(self,
                                     name: str,
                                     controllers: Sequence[BaseController]) -> str:
        if not controllers:
            raise ValueError("Cannot resolve combined plot path without controllers")

        storage = self._storage
        if storage is None:
            raise RuntimeError(
                "Combined plot storage requested but no data storage was provided to ControllerPlotter"
            )

        key = tuple(controller.abbreviation.lower() for controller in controllers)
        if list(controllers) == self._controllers:
            target_storage = storage
        else:
            if key not in self._subset_storage:
                folder_name = self._build_subset_folder_name(controllers)
                self._subset_storage[key] = storage.create_child_storage(folder_name)
            target_storage = self._subset_storage[key]

        return target_storage.get_path(name, 'png')

    @staticmethod
    def _build_subset_folder_name(controllers: Sequence[BaseController]) -> str:
        def sanitize(value: str) -> str:
            return "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in value.lower())

        parts = []
        for index, controller in enumerate(controllers, start=1):
            parts.append(f"{index:02d}-{sanitize(controller.abbreviation)}")
        return "combined_" + "_".join(parts)
