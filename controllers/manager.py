from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from tools.dataStorage import ControllerAssignment, DataStorage

from .BaseController import BaseController
from . import create_instance, get_controller_type


@dataclass
class AggregatedMetric:
    """Container describing an aggregated controller metric."""

    data: np.ndarray
    vehicle_serials: List[int]

    def as_dict(self) -> Dict[int, np.ndarray]:
        """Return a mapping from vehicle serial numbers to metric rows."""
        return {serial: self.data[idx] for idx, serial in enumerate(self.vehicle_serials)}


class ControllerManager:
    """Factory and coordinator for controllers within a simulation cycle."""

    def __init__(self,
                 controller_vehicle_groups: Iterable[Tuple[ControllerAssignment, Sequence[object]]],
                 arguments) -> None:
        self._controller_vehicle_groups = list(controller_vehicle_groups)
        self._arguments = arguments
        self._controllers: List[BaseController] = []
        self.controller_runs: List[Tuple[BaseController, Optional[object]]] = []
        self._vehicle_serials: Dict[BaseController, List[int]] = {}
        self._data_storage: Optional[DataStorage] = None

    @property
    def controllers(self) -> List[BaseController]:
        return list(self._controllers)

    @property
    def data_storage(self) -> Optional[DataStorage]:
        return self._data_storage

    def initialize_controllers(self,
                               space,
                               data_storage: DataStorage) -> List[Tuple[BaseController, Optional[object]]]:
        """Instantiate controllers and attach dedicated data storage objects."""

        self._controllers.clear()
        self.controller_runs.clear()
        self._vehicle_serials.clear()

        self._data_storage = data_storage.create_child_storage("controllers")

        for assignment_index, (assignment, vehicles) in enumerate(self._controller_vehicle_groups, start=1):
            controller_mode = get_controller_type(assignment.controller_type)
            if controller_mode == "swarm":
                controller = self._instantiate_controller(assignment.controller_type, vehicles, space)
                self._register_controller(controller, assignment_index, vehicles)
            elif controller_mode == "individual":
                for vehicle in vehicles:
                    controller = self._instantiate_controller(assignment.controller_type, [vehicle], space)
                    self._register_controller(controller, assignment_index, [vehicle])
            else:
                raise ValueError(f"Unsupported controller type: {controller_mode}")

        return self.controller_runs

    def set_run_result(self, index: int, result) -> None:
        controller, _ = self.controller_runs[index]
        self.controller_runs[index] = (controller, result)

    def aggregate_metric(self, metric_name: str) -> AggregatedMetric:
        attribute = metric_name
        aggregated: List[np.ndarray] = []
        serials: List[int] = []

        for controller in self._controllers:
            if not hasattr(controller, attribute):
                raise AttributeError(f"Controller '{controller.name}' has no attribute '{attribute}'")

            data = getattr(controller, attribute)
            array = np.asarray(data)
            if array.ndim == 1:
                array = np.expand_dims(array, axis=0)

            expected_rows = len(self._vehicle_serials[controller])
            if array.shape[0] != expected_rows:
                raise ValueError(
                    f"Metric '{attribute}' for controller '{controller.name}' has {array.shape[0]} rows, "
                    f"but {expected_rows} vehicles are registered"
                )

            aggregated.append(array)
            serials.extend(self._vehicle_serials[controller])

        if not aggregated:
            return AggregatedMetric(data=np.empty((0, 0)), vehicle_serials=[])

        concatenated = np.concatenate(aggregated, axis=0)
        return AggregatedMetric(data=concatenated, vehicle_serials=serials)

    def _instantiate_controller(self, controller_name: str, vehicles, space) -> BaseController:
        arguments = self._arguments
        return create_instance(controller_name,
                               vehicles=vehicles,
                               sim_time=arguments.sim_time_sec,
                               sample_time=arguments.sample_time,
                               space=space,
                               FPS=arguments.FPS,
                               isolines=arguments.isolines,
                               eps=arguments.eps,
                               e_max_cap=arguments.error_max_cap,
                               dynamic_error_max=arguments.dynamic_error_max,
                               smoothing=arguments.smoothing,
                               plot_config_path=arguments.plot_config,
                               use_latex=getattr(arguments, 'use_latex', True))

    def _register_controller(self,
                             controller: BaseController,
                             assignment_index: int,
                             vehicles) -> None:
        if self._data_storage is None:
            raise RuntimeError("Data storage must be configured before registering controllers")

        storage_name = self._build_storage_name(controller, assignment_index, vehicles)
        controller_storage = self._data_storage.create_child_storage(storage_name)
        controller.set_data_storage(controller_storage)

        self._controllers.append(controller)
        self.controller_runs.append((controller, None))
        self._vehicle_serials[controller] = [vehicle.serial_number for vehicle in vehicles]

        print(controller)
        print(f"Controller storage: {controller_storage.timestamped_folder}")

    @staticmethod
    def _build_storage_name(controller: BaseController,
                            assignment_index: int,
                            vehicles) -> str:
        parts = [controller.abbreviation.lower(), f"a{assignment_index:02d}"]
        if len(vehicles) == 1:
            parts.append(f"v{vehicles[0].serial_number:03d}")
        else:
            parts.append(f"g{len(vehicles):02d}")
        return "_".join(parts)
