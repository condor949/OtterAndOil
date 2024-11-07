from typing import Sequence

from spaces import BaseSpace
import numpy as np
import matplotlib.pyplot as plt


class Gaussian3DSpace(BaseSpace):

    def __init__(self, x_range=(-30, 30), y_range=(-30, 30), grid_size=500, shift_x=0, shift_y=0):
        """
        Initialize the 3D Gaussian space.

        Parameters:
        x_range (tuple): Range of x-axis values.
        y_range (tuple): Range of y-axis values.
        grid_size (int): Number of points in each dimension.
        shift_x (int): Shift of all points along the x-axis.
        shift_y (int): Shift of all points along the y-axis.
        """
        super().__init__()
        self.x = np.linspace(*x_range, grid_size)
        self.y = np.linspace(*y_range, grid_size)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        self.Z = np.zeros_like(self.X)  # Start with a flat surface
        self.shift_x = shift_x
        self.shift_y = shift_y

    def add_gaussian_peak(self, x0, y0, amplitude, sigma_x, sigma_y):
        peak = amplitude - ((self.X - x0 - self.shift_x) ** 2 / (2 * sigma_x) + (self.Y - y0 - self.shift_y) ** 2 / (2 * sigma_y))
        self.Z += peak
    # def add_gaussian_peak(self, x0, y0, amplitude, sigma_x, sigma_y):
    #     """
    #     Add a Gaussian peak to the Z surface.
    #
    #     Parameters:
    #     x0, y0 (float): Center of the Gaussian peak.
    #     amplitude (float): Height of the Gaussian peak.
    #     sigma_x, sigma_y (float): Spread of the Gaussian in x and y directions.
    #     """
    #     peak = amplitude * np.exp(-((self.X - x0 - self.shift_x) ** 2 / (2 * sigma_x ** 2) +
    #                                 (self.Y - y0 - self.shift_y) ** 2 / (2 * sigma_y ** 2)))
    #     self.Z += peak  # Add peak to the current Z surface

    def get_intensity(self, x_current, y_current):
        """
        Get the Z value at a specific (x, y) point.

        Parameters:
        x_current, y_current (float): The x and y coordinates for which to get the Z value.

        Returns:
        float: The Z value at the specified (x, y) point.
        """
        idx_x = (np.abs(self.x - x_current)).argmin()
        idx_y = (np.abs(self.y - y_current)).argmin()
        # print(self.Z[idx_x,idx_y])
        return self.Z[idx_x, idx_y]

    def drown(self, z_correction: float):
        self.Z-=z_correction

    def plot_surface(self, title="3D Gaussian Surface"):
        """
        Plot the 3D Gaussian surface with all peaks.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(self.X, self.Y, self.Z, cmap='viridis')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        plt.title(title)
        # plt.show()

    def get_X(self) -> Sequence:
        return self.X

    def get_Y(self) -> Sequence:
        return self.Y

    def get_Z(self) -> Sequence:
        return self.Z