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


def create_timestamped_folder(*args, base_path="./data", timestamped_suffix=""):
    """
    Creates a new folder with a name containing the current date and time.

    Parameters:
    - base_path (str): The base directory where the new folder will be created. Defaults to the current directory.

    Returns:
    - str: The path to the created folder.
    """

    # Construct the folder name
    folder_name = "experiment_"
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


def clean_data():
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


class Arguments:
    def __init__(self,
                 clean_cache: bool,
                 big_picture: bool,
                 not_animated: bool,
                 space_filename: str,
                 space_type: str,
                 N: int,
                 sample_time: float,
                 cycles: int,
                 radius: int,
                 catamarans: int,
                 grid_size: int,
                 FPS: int,
                 V_current: float,
                 beta_current: float):
        self.clean_cache = clean_cache
        self.big_picture = big_picture
        self.not_animated = not_animated
        self.space_filename = space_filename
        self.space_type = space_type
        self.N = N
        self.sample_time = sample_time
        self.cycles = cycles
        self.radius = radius
        self.catamarans = catamarans
        self.grid_size = grid_size
        self.FPS = FPS
        self.V_current = V_current
        self.beta_current = beta_current

    def get_json_data(self):
        return {
                    "clean_cache": self.clean_cache,
                    "big_picture": self.big_picture,
                    "not_animated": self.not_animated,
                    "space_filename": self.space_filename,
                    "space_type": self.space_type,
                    "N": self.N,
                    "sample_time": self.sample_time,
                    "cycles": self.cycles,
                    "radius": self.radius,
                    "catamarans": self.catamarans,
                    "grid_size": self.grid_size,
                    "FPS": self.FPS,
                    "V_current": self.V_current,
                    "beta_current": self.beta_current
                }

    # Save the variables to a new JSON file
    def store_in_config(self, folder, suffix):
        with open(os.path.join(folder,
                               create_timestamped_filename_ext("config",
                                                               suffix,
                                                               "json")), 'w') as config:
            json.dump(self.get_json_data(), config, indent=4)


def read_and_assign_parameters(input_filename):
    with open(input_filename, 'r') as file:
        data = json.load(file)

    # Assign values with appropriate types
    return Arguments(clean_cache = bool(data['clean_cache']),
                     big_picture = bool(data['big_picture']),
                     not_animated = bool(data['not_animated']),
                     space_filename = str(data['space_filename']),
                     space_type = str(data['space_type']),
                     N = int(data['N']),
                     sample_time = float(data['sample_time']),
                     cycles = int(data['cycles']),
                     radius = int(data['radius']),
                     catamarans = int(data['catamarans']),
                     grid_size=int(data['grid_size']),
                     FPS=int(data['FPS']),
                     V_current = float(data['V_current']),
                     beta_current = float(data['beta_current']))


def overwrite_file(old_name, new_name):
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


