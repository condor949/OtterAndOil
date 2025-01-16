import os
import datetime
import shutil
import json


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

    return folder_path


def clean_data() -> None:
    """
        Deletes the 'data' folder in the script's directory if it exists.
    """
    # Get the path of the script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Path to the 'data' folder
    data_folder_path = os.path.join(script_dir, "..", "data")

    # Check if the 'data' folder exists
    if os.path.exists(data_folder_path) and os.path.isdir(data_folder_path):
        # Delete the 'data' folder and its contents
        shutil.rmtree(data_folder_path)
        print(f"Deleted 'data' folder at: {data_folder_path}")
    else:
        print(f"'data' folder does not exist at: {data_folder_path}")


class DataStorage:
    def __init__(self, typename, series):
        self.timestamped_suffix = create_timestamped_suffix()
        self.timestamped_folder = create_timestamped_folder(typename,
                                                            f"s{series + 1}",
                                                            timestamped_suffix=self.timestamped_suffix)

    def __str__(self):
        return (f'Result folder: {self.timestamped_folder}')

    def get_path(self, name, expansion) -> str:
        return os.path.join(self.timestamped_folder,
                            create_timestamped_filename_ext(name,
                                                            self.timestamped_suffix,
                                                            expansion))

class Arguments:
    def __init__(self, **arguments):
        for key, value in arguments.items():
            setattr(self, key, value)
        self.data_storage = None

    def get_json_data(self):
        return {
                    "clean_cache": self.clean_cache,
                    "big_picture": self.big_picture,
                    "not_animated": self.not_animated,
                    "store_raw": self.store_raw,
                    "show_intensity": self.show_intensity,
                    "axis_abs_max": self.axis_abs_max,
                    "isometric": self.isometric,
                    "isolines": self.isolines,
                    "peaks_filename": self.peaks_filename,
                    "cache_dir": self.cache_dir,
                    "peak_type": self.peak_type,
                    "controller_type": self.controller_type,
                    "vehicle_types": self.vehicle_types,
                    "shift_vehicle": self.shift_vehicle,
                    "start_points": self.start_points,
                    "shift_xyz": self.shift_xyz,
                    "target_isoline": self.target_isoline,
                    "sim_time_sec": self.sim_time_sec,
                    "sample_time": self.sample_time,
                    "cycles": self.cycles,
                    "radius": self.radius,
                    "vehicles": self.vehicles,
                    "grid_size": self.grid_size,
                    "FPS": self.FPS,
                    "V_current": self.V_current,
                    "beta_current": self.beta_current
                }

    # Save the variables to a new JSON file
    def store_in_config(self) -> None:
        with open(self.data_storage.get_path("config", "json"), 'w') as config:
            json.dump(self.get_json_data(), config, indent=4)

    def set_data_storage(self, data_storage: DataStorage) -> None:
        self.data_storage = data_storage


def read_and_assign_arguments(input_filename) -> Arguments:
    with open(input_filename, 'r') as file:
        data = json.load(file)
    return Arguments(**data)


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


