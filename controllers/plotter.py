import json
import logging
import shutil
from collections.abc import Iterable as IterableCollection
from dataclasses import dataclass
from functools import partial
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rc, rcParams, rcParamsDefault
from tqdm import tqdm

from tools.dataStorage import DataStorage
from tools.random_generators import color_generator

from .BaseController import BaseController

logger = logging.getLogger(__name__)

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
        self._use_latex = bool(use_latex)
        if self._use_latex and shutil.which('latex') is None:
            self._use_latex = False
            logger.warning("LaTeX executable not found; falling back to Matplotlib text rendering without TeX.")
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
                       *,
                       controllers: Optional[Sequence[BaseController]] = None,
                       combine: bool = False) -> None:
        target_controllers = list(controllers) if controllers is not None else self._controllers
        if not target_controllers:
            raise ValueError("No controllers are available for track plotting")

        entries: List[Tuple[BaseController, IterableCollection, Dict[str, Any]]] = []
        for controller in target_controllers:
            sim_data = getattr(controller, "sim_data", None)
            if sim_data is None:
                raise ValueError(
                    f"Controller '{controller.name}' has no simulation data. Run the simulation before plotting tracks."
                )
            if not isinstance(sim_data, IterableCollection):
                raise TypeError("Controller simulation data must be an iterable of arrays for track plotting")
            metadata = self._extract_track_metadata(controller)
            entries.append((controller, sim_data, metadata))

        if combine:
            self._render_tracks(entries, combined=True)
        else:
            for entry in entries:
                self._render_tracks([entry], combined=False)

    def _render_tracks(self,
                       entries: Sequence[Tuple[BaseController, IterableCollection, Dict[str, Any]]],
                       *,
                       combined: bool) -> None:
        if not entries:
            return

        controllers = [controller for controller, _sim, _meta in entries]
        reference_metadata = entries[0][2]
        
        # Check if this is a simple track (without intensity field)
        is_simple_track = reference_metadata.get("simple_track", False)
        
        if combined:
            for controller, _sim, candidate_metadata in entries[1:]:
                # For simple tracks, skip compatibility check
                if not is_simple_track and not candidate_metadata.get("simple_track", False):
                    self._ensure_compatible_track_metadata(reference_metadata, candidate_metadata, controller)

        store_plot = self._track_options.store_plot
        animate = store_plot and not self._track_options.not_animated
        has_ffmpeg = shutil.which('ffmpeg') is not None
        if animate and not has_ffmpeg:
            animate = False
            if combined:
                logger.warning("Skipping combined track animation: 'ffmpeg' executable not found")
            else:
                logger.warning("Skipping track animation for %s: 'ffmpeg' executable not found",
                               getattr(controllers[0], 'name', 'controller'))

        if is_simple_track:
            # Simple track rendering without intensity field
            grid_size = reference_metadata.get("grid_size", 50)
            fps = reference_metadata.get("fps", 30)
        else:
            # Standard track rendering with intensity field
            grid_x = reference_metadata["grid_x"]
            grid_y = reference_metadata["grid_y"]
            grid_z = reference_metadata["grid_z"]
            grid_size = reference_metadata["grid_size"]
            target_isoline = reference_metadata["target_isoline"]
            isolines = reference_metadata["isolines"]
            fps = reference_metadata["fps"]

        if self._track_options.big_picture:
            fig = plt.figure(figsize=(cm2inch(bigFigSize1[0]), cm2inch(bigFigSize1[1])),
                             dpi=self._dpi)
        else:
            fig = plt.figure(figsize=(cm2inch(figSize1[0]), cm2inch(figSize1[1])),
                             dpi=self._dpi)

        plt.rc('text', usetex=self._use_latex)
        rc('font', size=30)
        
        if is_simple_track:
            # Simple track - no intensity field
            plt.xlabel('X, m / East')
            plt.ylabel('Y, m / North')
            plt.grid(True, alpha=0.3)
            plt.gca().set_aspect('equal', adjustable='box')
        else:
            # Standard track with intensity field
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
                    prev_index = max(num - 1, 0)
                    dx = dataSet[0, num] - dataSet[0, prev_index]
                    dy = dataSet[1, num] - dataSet[1, prev_index]

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
        for controller, sim_data, metadata in entries:
            provided_colors = metadata["provided_colors"]
            for i, vehicle_data in enumerate(sim_data):
                data_array = np.asarray(vehicle_data)
                if data_array.ndim != 2 or data_array.shape[1] < 3:
                    raise ValueError("Simulation data must be a 2D array with at least three columns (x, y, z)")

                x = data_array[:, 0]
                y = data_array[:, 1]
                z = data_array[:, 2]

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

                # Use serial_number from vehicle instead of index
                if i < len(controller.vehicles):
                    vehicle_serial = controller.vehicles[i].serial_number
                    label_suffix = f'vehicle {vehicle_serial}'
                else:
                    # Fallback to index if vehicle not available
                    label_suffix = f'vehicle {i + 1}'
                
                if combined:
                    controller_label = getattr(controller, 'abbreviation', None) or controller.name
                    label = f'{controller_label} {label_suffix}'
                else:
                    label = label_suffix
                line = plt.plot(dataSet[0], dataSet[1], lw=2, c=color, zorder=10, label=label)[0]
                plotData[line] = dataSet

        plt.legend()

        if store_plot:
            plt.tight_layout()
            if combined:
                output_path = self._resolve_combined_plot_path('track', controllers)
            else:
                output_path = controllers[0].get_plot_path('track', 'png')
            plt.savefig(output_path)
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

                if combined:
                    animation_path = self._resolve_combined_plot_path('track', controllers, extension='gif')
                else:
                    animation_path = controllers[0].get_plot_path('track', 'gif')
                ani.save(animation_path,
                         writer=writer,
                         progress_callback=update_func)

        plt.close(fig)

    def _extract_track_metadata(self, controller: BaseController) -> Dict[str, Any]:
        track_snapshot = controller.track_snapshot()
        if not isinstance(track_snapshot, dict):
            raise TypeError("track_snapshot must return a mapping with track metadata")

        # Check if this is a simple track (without intensity field)
        is_simple_track = track_snapshot.get("simple_track", False)
        
        if is_simple_track:
            # Simple tracks only need: grid_size, fps, colors
            required_keys = ["grid_size", "fps"]
        else:
            # Standard tracks need all keys including space data
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

        # Extract metadata based on track type
        if is_simple_track:
            # Simple track - only extract basic info
            grid_size = int(track_snapshot["grid_size"])
            fps = int(track_snapshot.get("fps", 30))
            provided_colors = track_snapshot.get("colors")
            return {
                "simple_track": True,
                "grid_size": grid_size,
                "fps": fps,
                "provided_colors": provided_colors,
            }
        else:
            # Standard track - extract all space-related data
            grid_x = np.asarray(track_snapshot["grid_x"])
            grid_y = np.asarray(track_snapshot["grid_y"])
            grid_z = np.asarray(track_snapshot["grid_z"])
            grid_size = int(track_snapshot["grid_size"])
            target_isoline = track_snapshot["target_isoline"]
            isoline_values = track_snapshot["isolines"]
            if np.isscalar(isoline_values):
                isolines = isoline_values
            else:
                isolines = np.asarray(isoline_values)
                if isolines.ndim == 0:
                    isolines = isolines.reshape(1)
                elif isolines.ndim > 1:
                    isolines = isolines.reshape(-1)
            fps = int(track_snapshot["fps"])
            colors_metadata = track_snapshot.get("colors")
            provided_colors = list(colors_metadata) if colors_metadata is not None else []

            if grid_size <= 0:
                raise ValueError("Track snapshot must define a positive grid size")

            return {
                "grid_x": grid_x,
                "grid_y": grid_y,
                "grid_z": grid_z,
                "grid_size": grid_size,
                "target_isoline": target_isoline,
                "isolines": isolines,
                "fps": fps,
                "provided_colors": provided_colors,
            }

    def _ensure_compatible_track_metadata(self,
                                          reference: Dict[str, Any],
                                          candidate: Dict[str, Any],
                                          controller: BaseController) -> None:
        controller_name = getattr(controller, 'name', 'controller')
        if reference["grid_size"] != candidate["grid_size"]:
            raise ValueError(
                f"Controller '{controller_name}' has a different grid_size and cannot be combined"
            )

        if reference["fps"] != candidate["fps"]:
            raise ValueError(
                f"Controller '{controller_name}' has a different fps and cannot be combined"
            )

        if not np.allclose(reference["grid_x"], candidate["grid_x"]):
            raise ValueError(
                f"Controller '{controller_name}' has different grid_x values and cannot be combined"
            )

        if not np.allclose(reference["grid_y"], candidate["grid_y"]):
            raise ValueError(
                f"Controller '{controller_name}' has different grid_y values and cannot be combined"
            )

        if not np.allclose(reference["grid_z"], candidate["grid_z"]):
            raise ValueError(
                f"Controller '{controller_name}' has different grid_z values and cannot be combined"
            )

        reference_isolines = reference["isolines"]
        candidate_isolines = candidate["isolines"]
        if np.isscalar(reference_isolines) and np.isscalar(candidate_isolines):
            if not np.isclose(reference_isolines, candidate_isolines):
                raise ValueError(
                    f"Controller '{controller_name}' has different isolines and cannot be combined"
                )
        elif np.isscalar(reference_isolines) != np.isscalar(candidate_isolines):
            raise ValueError(
                f"Controller '{controller_name}' mixes scalar and array isolines and cannot be combined"
            )
        else:
            if not np.allclose(np.asarray(reference_isolines), np.asarray(candidate_isolines)):
                raise ValueError(
                    f"Controller '{controller_name}' has different isolines and cannot be combined"
                )

        reference_isoline = reference["target_isoline"]
        candidate_isoline = candidate["target_isoline"]
        if isinstance(reference_isoline, (int, float)) and isinstance(candidate_isoline, (int, float)):
            if not np.isclose(reference_isoline, candidate_isoline):
                raise ValueError(
                    f"Controller '{controller_name}' has a different target_isoline and cannot be combined"
                )
        elif reference_isoline != candidate_isoline:
            raise ValueError(
                f"Controller '{controller_name}' has a different target_isoline and cannot be combined"
            )

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
                                     controllers: Sequence[BaseController],
                                     extension: str = 'png') -> str:
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

        return target_storage.get_path(name, extension)

    def plotting_control(self,
                        *,
                        controllers: Optional[Sequence[BaseController]] = None,
                        combine: bool = False) -> None:
        """
        Plot control inputs (u_control and u_actual) for each vehicle.
        
        This method extracts control data from sim_data and plots it similar to
        PythonVehicleSimulator's plotControls function.
        """
        import math
        
        target_controllers = list(controllers) if controllers is not None else self._controllers
        if not target_controllers:
            raise ValueError("No controllers are available for plotting controls")
        
        store_plot = self._plot_options.store_plots
        
        for controller in target_controllers:
            if controller.sim_data is None:
                logger.warning(
                    "Controller '%s' has no simulation data. Run the simulation before plotting controls.",
                    controller.name
                )
                continue
            
            # Detect DOF from the first vehicle's data
            DOF = len(controller.vehicles[0].nu) if controller.vehicles else 6
            
            for vehicle_index, vehicle in enumerate(controller.vehicles):
                vehicle_data = controller.sim_data[vehicle_index]
                if vehicle_data is None or vehicle_data.size == 0:
                    continue
                
                # Extract time vector
                t = controller.simTime
                
                # Determine subplot layout
                col = 2
                row = int(math.ceil(vehicle.dimU / col))
                
                # Create figure
                fig_size = (30, 20) if self._plot_options.big_picture else (10, 6)
                fig = plt.figure(figsize=fig_size, dpi=self._dpi)
                
                # Plot each control input
                for i in range(vehicle.dimU):
                    # Extract control data: u_control and u_actual
                    # sim_data structure: [eta (DOF), nu (DOF), u_control (dimU), u_actual (dimU)]
                    u_control = vehicle_data[:, 2 * DOF + i]  # control input, commands
                    u_actual = vehicle_data[:, 2 * DOF + vehicle.dimU + i]  # actual control input
                    
                    # Convert angles to degrees if needed
                    if vehicle.controls[i].find("deg") != -1:
                        u_control = np.degrees(u_control)
                        u_actual = np.degrees(u_actual)
                    
                    plt.subplot(row, col, i + 1)
                    plt.plot(t, u_control, label=f"{vehicle.controls[i]}, command")
                    plt.plot(t, u_actual, label=f"{vehicle.controls[i]}, actual")
                    plt.legend(fontsize=10)
                    plt.xlabel("Time, s", fontsize=12)
                    plt.grid(True)
                
                plt.tight_layout()
                
                if store_plot:
                    output_path = controller.get_plot_path('control', 'png')
                    plt.savefig(output_path)
                    logger.info("Saved control plot to %s", output_path)
                else:
                    plt.show()
                plt.close()

    def plotting_yaw_rate(self,
                          *,
                          controllers: Optional[Sequence[BaseController]] = None,
                          combine: bool = False) -> None:
        """
        Plot yaw rate r(t) in a separate figure per controller/vehicle.
        sim_data layout: [eta (DOF), nu (DOF), ...]; r is last component of nu (rad/s), shown in deg/s.
        """
        target_controllers = list(controllers) if controllers is not None else self._controllers
        if not target_controllers:
            raise ValueError("No controllers are available for yaw rate plotting")
        store_plot = self._plot_options.store_plots
        for controller in target_controllers:
            if controller.sim_data is None:
                logger.warning(
                    "Controller '%s' has no simulation data. Run the simulation before plotting yaw rate.",
                    controller.name,
                )
                continue
            DOF = len(controller.vehicles[0].nu) if controller.vehicles else 3
            for vehicle_index, vehicle in enumerate(controller.vehicles):
                vehicle_data = controller.sim_data[vehicle_index]
                if vehicle_data is None or vehicle_data.size == 0:
                    continue
                N_rows = vehicle_data.shape[0]
                t = controller.simTime
                if len(t) != N_rows:
                    t = np.linspace(0, controller.sample_time * (N_rows - 1), N_rows)
                # r = nu[yaw_rate_nu_index]; column in row is DOF + that index
                r_nu_index = getattr(vehicle, 'yaw_rate_nu_index', DOF - 1)
                r_rad = vehicle_data[:, DOF + r_nu_index]
                r_deg_s = np.degrees(r_rad)
                fig = plt.figure(figsize=(10, 4), dpi=self._dpi)
                plt.plot(t, r_deg_s, label="Yaw rate")
                plt.xlabel("Time, s", fontsize=12)
                plt.ylabel("Yaw rate, deg/s", fontsize=12)
                plt.title("Yaw rate vs time", fontsize=12)
                plt.legend(fontsize=10)
                plt.grid(True)
                plt.tight_layout()
                if store_plot:
                    output_path = controller.get_plot_path('yaw_rate', 'png')
                    plt.savefig(output_path)
                    logger.info("Saved yaw rate plot to %s", output_path)
                else:
                    plt.show()
                plt.close()

    def plotting_surge_velocity(self,
                                *,
                                controllers: Optional[Sequence[BaseController]] = None,
                                combine: bool = False) -> None:
        """
        Plot surge velocity u(t) in a separate figure per controller/vehicle.
        sim_data layout: [eta (DOF), nu (DOF), ...]; u is first component of nu (m/s).
        """
        target_controllers = list(controllers) if controllers is not None else self._controllers
        if not target_controllers:
            raise ValueError("No controllers are available for surge velocity plotting")
        store_plot = self._plot_options.store_plots
        for controller in target_controllers:
            if controller.sim_data is None:
                logger.warning(
                    "Controller '%s' has no simulation data. Run the simulation before plotting surge velocity.",
                    controller.name,
                )
                continue
            DOF = len(controller.vehicles[0].nu) if controller.vehicles else 3
            for vehicle_index, vehicle in enumerate(controller.vehicles):
                vehicle_data = controller.sim_data[vehicle_index]
                if vehicle_data is None or vehicle_data.size == 0:
                    continue
                N_rows = vehicle_data.shape[0]
                t = controller.simTime
                if len(t) != N_rows:
                    t = np.linspace(0, controller.sample_time * (N_rows - 1), N_rows)
                surge_indices = getattr(vehicle, 'surge_velocity_nu_indices', None)
                if surge_indices is not None:
                    # Surge = magnitude of velocity components (e.g. Dubins: sqrt(dx^2 + dy^2))
                    u = np.sqrt(
                        np.sum(vehicle_data[:, DOF + np.asarray(surge_indices)] ** 2, axis=1)
                    )
                else:
                    u = vehicle_data[:, DOF]
                fig = plt.figure(figsize=(10, 4), dpi=self._dpi)
                plt.plot(t, u, label="Surge velocity")
                plt.xlabel("Time, s", fontsize=12)
                plt.ylabel("Surge velocity, m/s", fontsize=12)
                plt.title("Surge velocity vs time", fontsize=12)
                plt.legend(fontsize=10)
                plt.grid(True)
                plt.tight_layout()
                if store_plot:
                    output_path = controller.get_plot_path('surge_velocity', 'png')
                    plt.savefig(output_path)
                    logger.info("Saved surge velocity plot to %s", output_path)
                else:
                    plt.show()
                plt.close()

    def plotting_yaw_rate_comparison(self,
                                     *,
                                     controllers: Optional[Sequence[BaseController]] = None) -> None:
        """
        Compare Dubins kinematic turning rate (ω_cmd) and Otter dynamic yaw rate (r)
        on one plot. Dubins ω_cmd is a kinematic turning rate from (n2-n1)*R/B;
        Otter r is a dynamic state from nu. Both shown in deg/s, single time axis.
        """
        target = list(controllers) if controllers is not None else self._controllers
        if not target:
            raise ValueError("No controllers available for yaw rate comparison")
        store_plot = self._plot_options.store_plots

        dubins_tuple = None
        otter_tuple = None
        for controller in target:
            if controller.sim_data is None:
                continue
            for vehicle_index, vehicle in enumerate(controller.vehicles):
                if getattr(vehicle, 'name', None) == 'dubins' and dubins_tuple is None:
                    dubins_tuple = (controller, vehicle_index, vehicle)
                if getattr(vehicle, 'name', None) in ('otter3d', 'otter') and otter_tuple is None:
                    otter_tuple = (controller, vehicle_index, vehicle)
            if dubins_tuple and otter_tuple:
                break

        if dubins_tuple is None or otter_tuple is None:
            logger.warning(
                "Yaw rate comparison skipped: need both a Dubins and an Otter vehicle. "
                "Dubins=%s, Otter=%s",
                dubins_tuple is not None,
                otter_tuple is not None,
            )
            return

        c_dubins, vi_dubins, vehicle_dubins = dubins_tuple
        c_otter, vi_otter, vehicle_otter = otter_tuple
        data_dubins = c_dubins.sim_data[vi_dubins]
        data_otter = c_otter.sim_data[vi_otter]
        if data_dubins is None or data_dubins.size == 0 or data_otter is None or data_otter.size == 0:
            logger.warning("Yaw rate comparison skipped: missing sim_data for Dubins or Otter")
            return

        DOF_d = len(vehicle_dubins.nu)
        DOF_o = len(vehicle_otter.nu)
        # sim_data row: [eta (DOF), nu (DOF), u_control (dimU), u_actual (dimU)]
        u_control_start = 2 * DOF_d
        n1 = data_dubins[:, u_control_start]
        n2 = data_dubins[:, u_control_start + 1]
        # ω_cmd (Dubins): kinematic rate from command, same formula as in dynamics (rad/s)
        R = getattr(vehicle_dubins, 'R', 0.315)
        B = getattr(vehicle_dubins, 'B', 1.0)
        omega_cmd_rad = (n2 - n1) * R / B

        r_nu_index = getattr(vehicle_otter, 'yaw_rate_nu_index', DOF_o - 1)
        r_rad = data_otter[:, DOF_o + r_nu_index]

        n_common = min(len(omega_cmd_rad), len(r_rad))
        if n_common == 0:
            logger.warning("Yaw rate comparison skipped: empty time series")
            return
        t = c_dubins.simTime
        if len(t) != n_common:
            t = np.linspace(0, c_dubins.sample_time * (n_common - 1), n_common)
        else:
            t = t[:n_common]
        omega_cmd_rad = omega_cmd_rad[:n_common]
        r_rad = r_rad[:n_common]
        omega_cmd_deg = np.degrees(omega_cmd_rad)
        r_deg = np.degrees(r_rad)

        # Dubins ω_cmd is a kinematic turning rate; Otter r is a dynamic state.
        fig = plt.figure(figsize=(10, 4), dpi=self._dpi)
        plt.plot(t, omega_cmd_deg, label="Dubins omega_cmd")
        plt.plot(t, r_deg, label="Otter r")
        plt.xlabel("Time (s)", fontsize=12)
        plt.ylabel("Yaw rate (deg/s)", fontsize=12)
        plt.title("Yaw rate comparison: Dubins command vs Otter dynamics", fontsize=12)
        plt.legend(fontsize=10)
        plt.grid(True)
        plt.tight_layout()
        if store_plot:
            output_path = c_dubins.get_plot_path('yaw_rate_comparison', 'png')
            plt.savefig(output_path)
            logger.info("Saved yaw rate comparison plot to %s", output_path)
        else:
            plt.show()
        plt.close()

    @staticmethod
    def _reconstruct_tau_from_u_actual(vehicle, u_actual: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Reconstruct tau_X and tau_N from u_actual using the same thrust/tau formulas
        as in vehicle.dynamics (e.g. Otter3D). Returns (tau_X, tau_N) or None if
        vehicle does not expose k_pos, k_neg, l1, l2.
        """
        k_pos = getattr(vehicle, 'k_pos', None)
        k_neg = getattr(vehicle, 'k_neg', None)
        l1 = getattr(vehicle, 'l1', None)
        l2 = getattr(vehicle, 'l2', None)
        if k_pos is None or k_neg is None or l1 is None or l2 is None:
            return None
        if u_actual.ndim == 1:
            n = u_actual.reshape(-1, 2)
        else:
            n = u_actual[:, :2]
        thrust = np.zeros_like(n)
        for i in range(2):
            pos = n[:, i] > 0
            thrust[:, i] = np.where(pos, k_pos * n[:, i] * np.abs(n[:, i]), k_neg * n[:, i] * np.abs(n[:, i]))
        tau_X = thrust[:, 0] + thrust[:, 1]
        tau_N = -float(l1) * thrust[:, 0] - float(l2) * thrust[:, 1]
        return tau_X, tau_N

    def plotting_tau_allocation(self,
                                *,
                                controllers: Optional[Sequence[BaseController]] = None,
                                combine: bool = False) -> None:
        """
        Plot tau_X(t), tau_N(t) and propeller speeds n1(t), n2(t) (command and actual)
        to verify the cascade: computed tau -> allocation -> n.
        """
        target_controllers = list(controllers) if controllers is not None else self._controllers
        if not target_controllers:
            raise ValueError("No controllers are available for tau_allocation plotting")
        store_plot = self._plot_options.store_plots

        if combine:
            self._plot_tau_allocation_combined(target_controllers, store_plot)
        else:
            for controller in target_controllers:
                self._plot_tau_allocation_single(controller, store_plot)

    def _plot_tau_allocation_single(self, controller: BaseController, store_plot: bool) -> None:
        if controller.sim_data is None:
            logger.warning(
                "Controller '%s' has no simulation data. Run the simulation before plotting tau_allocation.",
                controller.name,
            )
            return
        DOF = len(controller.vehicles[0].nu) if controller.vehicles else 3
        n_veh = controller.number_of_vehicles
        log_tau_X = getattr(controller, 'log_tau_X', None)
        log_tau_N = getattr(controller, 'log_tau_N', None)
        use_direct_tau = (
            log_tau_X is not None and log_tau_N is not None
            and len(log_tau_X) == n_veh * controller.N
            and len(log_tau_N) == n_veh * controller.N
        )
        if use_direct_tau:
            logger.info("tau_allocation: using direct tau from controller logging (log_tau_X, log_tau_N)")
        else:
            logger.info("tau_allocation: tau reconstructed from u_actual (vehicle thrust formulas)")

        for vehicle_index, vehicle in enumerate(controller.vehicles):
            vehicle_data = controller.sim_data[vehicle_index]
            if vehicle_data is None or vehicle_data.size == 0:
                continue
            N_rows = vehicle_data.shape[0]
            t = controller.simTime
            if len(t) != N_rows:
                t = np.linspace(0, controller.sample_time * (N_rows - 1), N_rows)

            u_control = vehicle_data[:, 2 * DOF : 2 * DOF + vehicle.dimU]
            u_actual = vehicle_data[:, 2 * DOF + vehicle.dimU : 2 * DOF + 2 * vehicle.dimU]

            if use_direct_tau:
                tau_X = np.array(log_tau_X[vehicle_index::n_veh], float)
                tau_N = np.array(log_tau_N[vehicle_index::n_veh], float)
                if len(tau_X) != N_rows:
                    tau_X = np.resize(tau_X, N_rows)
                    tau_N = np.resize(tau_N, N_rows)
            else:
                rec = self._reconstruct_tau_from_u_actual(vehicle, u_actual)
                if rec is None:
                    tau_X = np.full(N_rows, np.nan)
                    tau_N = np.full(N_rows, np.nan)
                else:
                    tau_X, tau_N = rec

            fig = plt.figure(figsize=(12, 10), dpi=self._dpi)
            ax1 = fig.add_subplot(2, 2, 1)
            ax1.plot(t, tau_X, 'b-')
            ax1.set_ylabel(r"$\tau_X$, N")
            ax1.set_xlabel("Time, s")
            ax1.grid(True)

            ax2 = fig.add_subplot(2, 2, 2)
            ax2.plot(t, tau_N, 'b-')
            ax2.set_ylabel(r"$\tau_N$, N·m")
            ax2.set_xlabel("Time, s")
            ax2.grid(True)

            ax3 = fig.add_subplot(2, 2, 3)
            ax3.plot(t, u_control[:, 0], '--', label="n1 command")
            ax3.plot(t, u_actual[:, 0], '-', label="n1 actual")
            ax3.set_ylabel("n1, rad/s")
            ax3.set_xlabel("Time, s")
            ax3.legend(fontsize=9)
            ax3.grid(True)

            ax4 = fig.add_subplot(2, 2, 4)
            ax4.plot(t, u_control[:, 1], '--', label="n2 command")
            ax4.plot(t, u_actual[:, 1], '-', label="n2 actual")
            ax4.set_ylabel("n2, rad/s")
            ax4.set_xlabel("Time, s")
            ax4.legend(fontsize=9)
            ax4.grid(True)

            plt.tight_layout()
            if store_plot:
                output_path = controller.get_plot_path('tau_allocation', 'png')
                plt.savefig(output_path)
                logger.info("Saved tau_allocation plot to %s", output_path)
            else:
                plt.show()
            plt.close()

    def _plot_tau_allocation_combined(self,
                                      controllers: Sequence[BaseController],
                                      store_plot: bool) -> None:
        color_gen = color_generator()
        fig = plt.figure(figsize=(12, 10), dpi=self._dpi)
        ax1 = fig.add_subplot(2, 2, 1)
        ax2 = fig.add_subplot(2, 2, 2)
        ax3 = fig.add_subplot(2, 2, 3)
        ax4 = fig.add_subplot(2, 2, 4)
        any_plotted = False

        for controller in controllers:
            if controller.sim_data is None:
                continue
            DOF = len(controller.vehicles[0].nu) if controller.vehicles else 3
            n_veh = controller.number_of_vehicles
            log_tau_X = getattr(controller, 'log_tau_X', None)
            log_tau_N = getattr(controller, 'log_tau_N', None)
            use_direct_tau = (
                log_tau_X is not None and log_tau_N is not None
                and len(log_tau_X) == n_veh * controller.N
                and len(log_tau_N) == n_veh * controller.N
            )
            color = next(color_gen)
            label_base = getattr(controller, 'abbreviation', None) or controller.name

            for vehicle_index, vehicle in enumerate(controller.vehicles):
                vehicle_data = controller.sim_data[vehicle_index]
                if vehicle_data is None or vehicle_data.size == 0:
                    continue
                N_rows = vehicle_data.shape[0]
                t = controller.simTime
                if len(t) != N_rows:
                    t = np.linspace(0, controller.sample_time * (N_rows - 1), N_rows)
                u_control = vehicle_data[:, 2 * DOF : 2 * DOF + vehicle.dimU]
                u_actual = vehicle_data[:, 2 * DOF + vehicle.dimU : 2 * DOF + 2 * vehicle.dimU]

                if use_direct_tau:
                    tau_X = np.array(log_tau_X[vehicle_index::n_veh], float)
                    tau_N = np.array(log_tau_N[vehicle_index::n_veh], float)
                    if len(tau_X) != N_rows:
                        tau_X = np.resize(tau_X, N_rows)
                        tau_N = np.resize(tau_N, N_rows)
                else:
                    rec = self._reconstruct_tau_from_u_actual(vehicle, u_actual)
                    if rec is None:
                        tau_X = np.full(N_rows, np.nan)
                        tau_N = np.full(N_rows, np.nan)
                    else:
                        tau_X, tau_N = rec

                suf = f" (v{vehicle_index + 1})" if n_veh > 1 else ""
                lbl = f"{label_base}{suf}"
                ax1.plot(t, tau_X, color=color, label=lbl)
                ax2.plot(t, tau_N, color=color, label=lbl)
                ax3.plot(t, u_actual[:, 0], '-', color=color, label=lbl)
                ax4.plot(t, u_actual[:, 1], '-', color=color, label=lbl)
                any_plotted = True

        _first = controllers[0] if controllers else None
        _has_log = (
            _first is not None
            and getattr(_first, "log_tau_X", None) is not None
            and len(_first.log_tau_X) == _first.number_of_vehicles * _first.N
        )
        logger.info(
            "tau_allocation (combined): using %s",
            "direct controller logging (log_tau_X, log_tau_N)" if _has_log else "tau reconstructed from u_actual",
        )

        ax1.set_ylabel(r"$\tau_X$, N")
        ax1.set_xlabel("Time, s")
        ax1.legend(fontsize=8)
        ax1.grid(True)
        ax2.set_ylabel(r"$\tau_N$, N·m")
        ax2.set_xlabel("Time, s")
        ax2.legend(fontsize=8)
        ax2.grid(True)
        ax3.set_ylabel("n1, rad/s")
        ax3.set_xlabel("Time, s")
        ax3.legend(fontsize=8)
        ax3.grid(True)
        ax4.set_ylabel("n2, rad/s")
        ax4.set_xlabel("Time, s")
        ax4.legend(fontsize=8)
        ax4.grid(True)
        plt.tight_layout()
        if store_plot and any_plotted:
            path = self._resolve_combined_plot_path('tau_allocation', controllers)
            plt.savefig(path)
            logger.info("Saved combined tau_allocation plot to %s", path)
        else:
            plt.show()
        plt.close()

    @staticmethod
    def _build_subset_folder_name(controllers: Sequence[BaseController]) -> str:
        def sanitize(value: str) -> str:
            return "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in value.lower())

        parts = []
        for index, controller in enumerate(controllers, start=1):
            parts.append(f"{index:02d}-{sanitize(controller.abbreviation)}")
        return "combined_" + "_".join(parts)
