import numpy as np
from spaces import BaseSpace


class Gaussian3DSpace(BaseSpace):
    def __init__(self, x_range=(-30, 30), y_range=(-30, 30), grid_size=500, shift_x=0, shift_y=0, shift_z=0, space_filename=""):
        """
        Initialize the 3D Gaussian space.

        Parameters:
        x_range (tuple): Range of x-axis values.
        y_range (tuple): Range of y-axis values.
        grid_size (int): Number of points in each dimension.
        shift_x (int): Shift of all points along the x-axis.
        shift_y (int): Shift of all points along the y-axis.
        """
        super().__init__(x_range, y_range, grid_size, shift_x, shift_y, shift_z, space_filename)
        """
            Add a Gaussian peak to the Z surface.

            Parameters:
            x0, y0 (float): Center of the Gaussian peak.
            amplitude (float): Height of the Gaussian peak.
            sigma_x, sigma_y (float): Spread of the Gaussian in x and y directions.
        """
        for peak in self.peaks:
            self.Z += peak.amplitude * np.exp(-((self.X - peak.x0 - self.shift_x) ** 2 / (2 * peak.sigma_x ** 2) +
                                                (self.Y - peak.y0 - self.shift_y) ** 2 / (2 * peak.sigma_y ** 2)))
        # print(self.Z)
        self.Z += self.shift_z
        self.type = "gaussian"

    def get_intensity(self, x_current, y_current):
        """
        Get the Z value at a specific (x, y) point.

        Parameters:
        x_current, y_current (float): The x and y coordinates for which to get the Z value.

        Returns:
        float: The Z value at the specified (x, y) point.
        """
        intensity: float = 0
        for peak in self.peaks:
            intensity += peak.amplitude * np.exp(-((x_current - peak.x0 - self.shift_x) ** 2 / (2 * peak.sigma_x ** 2) +
                                    (y_current - peak.y0 - self.shift_y) ** 2 / (2 * peak.sigma_y ** 2)))
        intensity += self.shift_z
        return intensity
