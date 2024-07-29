import os
import datetime
import shutil


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
