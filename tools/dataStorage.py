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


def create_timestamped_folder(base_path="./data", suffix=""):
    """
    Creates a new folder with a name containing the current date and time.

    Parameters:
    - base_path (str): The base directory where the new folder will be created. Defaults to the current directory.

    Returns:
    - str: The path to the created folder.
    """

    # Construct the folder name
    if len(suffix):
        folder_name = f"experiment_{suffix}"
    else:
        folder_name = f"experiment_{create_timestamped_suffix()}"

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
                 space_name: str,
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
        self.space_name = space_name
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
                    "space_name": self.space_name,
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


def read_and_assign_parameters(input_filename):
    with open(input_filename, 'r') as file:
        data = json.load(file)

    # Assign values with appropriate types
    return Arguments(clean_cache = bool(data['clean_cache']),
                     big_picture = bool(data['big_picture']),
                     not_animated = bool(data['not_animated']),
                     space_filename = str(data['space_filename']),
                     space_name = str(data['space_name']),
                     N = int(data['N']),
                     sample_time = float(data['sample_time']),
                     cycles = int(data['cycles']),
                     radius = int(data['radius']),
                     catamarans = int(data['catamarans']),
                     grid_size=int(data['grid_size']),
                     FPS=int(data['FPS']),
                     V_current = float(data['V_current']),
                     beta_current = float(data['beta_current']))


# Save the variables to a new JSON file
def save_parameters(output_filename, arguments: Arguments):
    with open(output_filename, 'w') as saved_config:
        json.dump(arguments.get_json_data(), saved_config, indent=4)


