import json
import numpy as np
from abc import ABC
from collections.abc import Sequence


class Peak:
    def __init__(self, x0, y0, amplitude, sigma_x, sigma_y):
        self.x0=x0
        self.y0=y0
        self.amplitude=amplitude
        self.sigma_x=sigma_x
        self.sigma_y=sigma_y

    def __repr__(self):
        return (f"Peak(x0={self.x0}, y0={self.y0}, amplitude={self.amplitude}, "
                f"sigma_x={self.sigma_x}, sigma_y={self.sigma_y})")


class BaseSpace(ABC):
    def __init__(self, x_range=(-30, 30), y_range=(-30, 30), grid_size=500, shift_x=0, shift_y=0, shift_z=0, space_filename=""):
        """
        Initialize the 3D space.

        Parameters:
        x_range (tuple): Range of x-axis values.
        y_range (tuple): Range of y-axis values.
        grid_size (int): Number of points in each dimension.
        shift_x (int): Shift of all points along the x-axis.
        shift_y (int): Shift of all points along the y-axis.
        """
        self.x = np.linspace(*x_range, grid_size)
        self.y = np.linspace(*y_range, grid_size)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        self.Z = np.zeros_like(self.X)  # Start with a flat surface
        self.shift_x = shift_x
        self.shift_y = shift_y
        self.shift_z = shift_z
        self.interp = None
        self.contour_points = list()
        self.peaks = list()
        if space_filename:
            with open(space_filename, 'r') as file:
                data = json.load(file)
                self.peaks = [Peak(**item) for item in data]

    def get_intensity(self, x_current, y_current):
        pass

    def plot_surface(self, path, title):
        pass

    def get_X(self) -> Sequence:
        return self.X

    def get_Y(self) -> Sequence:
        return self.Y

    def get_Z(self) -> Sequence:
        return self.Z
