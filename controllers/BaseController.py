import numpy as np
from abc import ABC
from spaces import BaseSpace
from collections.abc import Sequence


class BaseController(ABC):
    def __init__(self, timestamped_folder: str, timestamped_suffix:str, vehicles, sim_time: int, sample_time: float, space: BaseSpace):
        self.timestamped_folder = timestamped_folder
        self.timestamped_suffix = timestamped_suffix
        self.sample_time = sample_time
        self.sim_time = sim_time
        self.N = round(sim_time / sample_time) + 1
        self.simTime = np.arange(start=0, stop=self.sample_time*self.N, step=sample_time)#[:, None]
        self.space = space
        self.number_of_vehicles = len(vehicles)
        self.colors = {vehicle.serial_number: vehicle.color for vehicle in vehicles}

    def generate_control(self, vehicles, positions, step) -> Sequence:
        pass