import datetime
import json
import logging
import os
import shutil
from dataclasses import dataclass
from typing import Iterable, Iterator, List, Optional, Sequence, Tuple


logger = logging.getLogger(__name__)


@dataclass
class VehicleConfiguration:
    vehicle_type: str
    start_point: Sequence[float]


@dataclass
class ControllerAssignment:
    controller_type: str
    vehicles: List[VehicleConfiguration]

    def total_vehicles(self) -> int:
        return len(self.vehicles)

    def __str__(self):
        retval=(f"-------------------------------------\n"
                f"controller_type={self.controller_type}")
        for i, vehicle in enumerate(self.vehicles):
            retval=(f"{retval}\n"
                    f"vehicle #{i}\n"
                    f"vehicle_type={vehicle.vehicle_type}\n"
                    f"start_point={vehicle.start_point}\n")
        return retval


def _normalize_point(point: Sequence[float], field_name: str) -> Tuple[float, float]:
    if isinstance(point, (str, bytes)):
        raise TypeError(f"{field_name} must be a sequence of numeric values, got string-like input")
    try:
        x, y = point[0], point[1]
    except (TypeError, IndexError):
        raise ValueError(f"{field_name} must contain at least two numeric values") from None
    try:
        return float(x), float(y)
    except (TypeError, ValueError):
        raise ValueError(f"{field_name} values must be convertible to float") from None


def _resolve_start_points(count: int,
                          start_points: Optional[Sequence[Sequence[float]]],
                          field_name: str) -> List[Tuple[float, float]]:
    if start_points is None:
        return [(0.0, 0.0) for _ in range(count)]
    if isinstance(start_points, (str, bytes)):
        raise TypeError(f"{field_name} must be an array of point coordinates")
    if len(start_points) != count:
        raise ValueError(f"{field_name} must contain exactly {count} points, got {len(start_points)}")
    return [_normalize_point(point, field_name) for point in start_points]


class ControllerVehiclePairs(Iterable[ControllerAssignment]):
    """Container responsible for parsing and validating controller assignments."""

    def __init__(self, pairs: Optional[Sequence[dict]]):
        if pairs is None:
            raise ValueError("Configuration must define 'controller_vehicle_pairs'")
        if isinstance(pairs, (str, bytes)):
            raise TypeError("'controller_vehicle_pairs' must be an array of JSON objects")
        if not len(pairs):
            raise ValueError("'controller_vehicle_pairs' must contain at least one assignment")

        self._assignments: List[ControllerAssignment] = []
        self._sanitized: List[dict] = []

        logger.debug("Parsing %d controller assignment(s)", len(pairs))

        for idx, pair in enumerate(pairs):
            if not isinstance(pair, dict):
                raise TypeError(f"controller_vehicle_pairs[{idx}] must be a JSON object")

            controller_type = pair.get("controller_type")
            if not controller_type:
                raise ValueError(f"controller_vehicle_pairs[{idx}] must define 'controller_type'")

            if "vehicle_types" in pair:
                raise ValueError(
                    f"controller_vehicle_pairs[{idx}] must not define 'vehicle_types'; use a single 'vehicle_type' instead"
                )

            vehicle_type = pair.get("vehicle_type")
            if not vehicle_type:
                raise ValueError(f"controller_vehicle_pairs[{idx}] must define 'vehicle_type'")

            vehicles_count = pair.get("vehicles", 1)
            if not isinstance(vehicles_count, int) or vehicles_count <= 0:
                raise ValueError(f"controller_vehicle_pairs[{idx}]['vehicles'] must be a positive integer")

            start_points_entry = pair.get("start_points")
            start_points = _resolve_start_points(vehicles_count, start_points_entry,
                                                 f"controller_vehicle_pairs[{idx}]['start_points']")

            vehicles: List[VehicleConfiguration] = [
                VehicleConfiguration(vehicle_type=str(vehicle_type), start_point=start_point)
                for start_point in start_points
            ]

            self._assignments.append(ControllerAssignment(controller_type=str(controller_type),
                                                          vehicles=vehicles))

            sanitized_entry: dict = {
                "controller_type": str(controller_type),
                "vehicles": vehicles_count,
                "vehicle_type": str(vehicle_type)
            }
            if start_points_entry is not None:
                sanitized_entry["start_points"] = [list(point) for point in start_points]

            self._sanitized.append(sanitized_entry)

        total_vehicles = sum(assignment.total_vehicles() for assignment in self._assignments)
        logger.info(
            "Configured %d controller assignment(s) with %d total vehicle(s)",
            len(self._assignments),
            total_vehicles,
        )

    def __iter__(self) -> Iterator[ControllerAssignment]:
        return iter(self._assignments)

    def __len__(self) -> int:
        return len(self._assignments)

    @property
    def assignments(self) -> List[ControllerAssignment]:
        return list(self._assignments)

    @property
    def total_vehicles(self) -> int:
        return sum(assignment.total_vehicles() for assignment in self._assignments)

    def as_config(self) -> List[dict]:
        return [dict(entry) for entry in self._sanitized]


def create_timestamped_suffix() -> str:
    """
    Creates a filename for an experiment with the current date and time.

    Parameters:
    - Empty

    Returns:
    - str: The generated timestamp in YYYYMMDD_HHMMSS.
    """
    # Get the current date and time
    now = datetime.datetime.now()

    # Format the date and time as 'YYYYMMDD_HHMMSS'
    timestamp = now.strftime("%Y%m%d_%H%M%S")

    return timestamp


def create_timestamped_filename_ext(base_name: str, suffix: str, extension: str) -> str:
    if len(suffix):
        return f"{base_name}_{suffix}.{extension}"
    else:
        return f"{base_name}_{create_timestamped_suffix()}.{extension}"


def create_timestamped_folder(*args, base_path="./data", timestamped_suffix="") -> str:
    """
    Creates a new folder with a name containing the current date and time.

    Parameters:
    - base_path (str): The base directory where the new folder will be created. Defaults to the current directory.

    Returns:
    - str: The path to the created folder.
    """

    # Construct the folder name
    folder_name = "expt_"
    for arg in args:
        folder_name += f"{arg}_"
    if len(timestamped_suffix):
        folder_name += timestamped_suffix
    else:
        folder_name += create_timestamped_suffix()

    # Create the full path
    folder_path = os.path.join(base_path, folder_name)

    # Create the new folder
    os.makedirs(folder_path, exist_ok=True)

    logger.debug("Ensured data directory exists: %s", folder_path)

    return folder_path


def clean_data(cache_dir: str = "data") -> None:
    """Delete the directory used for cached experiment artifacts.

    Parameters
    ----------
    cache_dir:
        Directory containing cached results. Relative paths are resolved from
        the current working directory, matching how :class:`DataStorage`
        creates its output directories.
    """

    if not cache_dir:
        logger.warning("Empty cache_dir provided; skipping cache cleanup")
        return

    target_path = os.path.abspath(os.path.expanduser(cache_dir))
    if os.path.abspath(target_path) == os.path.abspath(os.sep):
        logger.warning("Refusing to remove cache directory at filesystem root: %s", target_path)
        return

    if os.path.isdir(target_path):
        shutil.rmtree(target_path)
        logger.info("Deleted cache directory at %s", target_path)
    else:
        logger.info("Cache directory does not exist at %s", target_path)


class DataStorage:
    def __init__(self, typename, series, cache_dir="data"):
        self.timestamped_suffix = create_timestamped_suffix()
        self.timestamped_folder = create_timestamped_folder(typename,
                                                            f"s{series + 1}",
                                                            timestamped_suffix=self.timestamped_suffix,
                                                            base_path=f"./{cache_dir}")
        logger.debug(
            "Initialized DataStorage for type '%s' (series %s) at %s",
            typename,
            series + 1,
            self.timestamped_folder,
        )

    def __str__(self):
        return f'Result folder: {self.timestamped_folder}'

    def get_path(self, name, expansion) -> str:
        path = os.path.join(self.timestamped_folder,
                            create_timestamped_filename_ext(name,
                                                            self.timestamped_suffix,
                                                            expansion))
        logger.debug("Resolved path for %s.%s -> %s", name, expansion, path)
        return path

    def create_child_storage(self, name: str) -> "DataStorage":
        child = DataStorage.__new__(DataStorage)
        child.timestamped_suffix = self.timestamped_suffix
        child.timestamped_folder = os.path.join(self.timestamped_folder, name)
        os.makedirs(child.timestamped_folder, exist_ok=True)
        logger.debug("Created child storage '%s' at %s", name, child.timestamped_folder)
        return child


class Arguments:
    def __init__(self, **arguments):
        for key, value in arguments.items():
            setattr(self, key, value)
        self.data_storage = None

    def get_json_data(self):
        excluded_keys = {"data_storage"}
        data = {}
        for key, value in vars(self).items():
            if key in excluded_keys:
                continue
            if value is None:
                continue
            data[key] = value
        return data

    # Save the variables to a new JSON file
    def store_in_config(self) -> None:
        with open(self.data_storage.get_path("config", "json"), 'w') as config:
            json.dump(self.get_json_data(), config, indent=4)

    def set_data_storage(self, data_storage: DataStorage) -> None:
        self.data_storage = data_storage


def read_and_assign_arguments(input_filename) -> Arguments:
    logger.info("Loading configuration from %s", input_filename)
    with open(input_filename, 'r') as file:
        data = json.load(file)

    arguments = Arguments(**data)
    logger.debug("Loaded configuration keys: %s", ", ".join(sorted(data.keys())))
    return arguments


def overwrite_file(old_name, new_name) -> None:
    """
    Overwrite a file with a different name.

    Parameters:
    old_name (str): The name of the file to be overwritten.
    new_name (str): The new name of the file.
    """
    # Check if the old file exists
    if not os.path.exists(old_name):
        raise FileNotFoundError(f"The file '{old_name}' does not exist.")

    # Remove the new file if it already exists
    if os.path.exists(new_name):
        os.remove(new_name)

    # Copy the old file to the new file name (overwriting if exists)
    shutil.copyfile(old_name, new_name)


