import numpy as np
from abc import ABC
from spaces import BaseSpace
from collections.abc import Sequence


class BaseController(ABC):
    name = 'base_controller'
    def __init__(self, vehicles, sim_time: int, sample_time: float, space: BaseSpace):
        self.sample_time = sample_time
        self.sim_time = sim_time
        self.N = round(sim_time / sample_time) + 1
        self.simTime = np.arange(start=0, stop=self.sample_time*self.N, step=sample_time)#[:, None]
        self.space = space
        self.number_of_vehicles = len(vehicles)
        self.vehicles = vehicles
        self.colors = {vehicle.serial_number: vehicle.color for vehicle in vehicles}
        self.data_storage = None

    def generate_control(self, positions, step) -> Sequence:
        pass

    def set_data_storage(self, data_storage) -> None:
        self.data_storage = data_storage