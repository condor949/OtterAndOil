import numpy as np
from spaces import BaseSpace


class Parabolic3DSpace(BaseSpace):
    name = 'parabolic'
    def __init__(self, x_range=(-50, 50), y_range=(-50, 50), grid_size=500, shift_xyz=None, space_filename="", target_isoline=0):
        """
        Initialize the 3D Parabolic space.

        Parameters:
        x_range (tuple): Range of x-axis values.
        y_range (tuple): Range of y-axis values.
        grid_size (int): Number of points in each dimension.
        shift_xyz (int): Shift of all points of space by values from this array, respectively XYZ.
        """
        super().__init__(x_range, y_range, grid_size, shift_xyz, space_filename, target_isoline)
        """
            Add a Parabolic peak to the Z surface.

            Parameters:
            x0, y0 (float): Center of the Parabolic peak.
            amplitude (float): Height of the Parabolic peak.
            sigma_x, sigma_y (float): Spread of the Parabolic in x and y directions.
        """
        for peak in self.peaks:
            self.Z += ((peak.amplitude -((self.X - peak.x0 - self.shift_xyz.shift_x()) ** 2 / (2 * peak.sigma_x ** 2) +
                                        (self.Y - peak.y0 - self.shift_xyz.shift_y()) ** 2 / (2 * peak.sigma_y ** 2))) *
                       np.exp(-((self.X - peak.x0 - self.shift_xyz.shift_x()) ** 2 / (2 * peak.sigma_x ** 2) +
                                (self.Y - peak.y0 - self.shift_xyz.shift_y()) ** 2 / (2 * peak.sigma_y ** 2))))
        # Corrects the error if the graph has a concave part below zero. All points below zero become zero.
        for i in range(len(self.X)):
            for j in range(len(self.X[i])):
                if self.Z[i, j] < 0:
                    self.Z[i, j] = 0
        #self.Z += self.shift_xyz.shift_z()
        self.type = "parabolic"

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
            intensity += ((peak.amplitude -((x_current - peak.x0 - self.shift_xyz.shift_x()) ** 2 / (2 * peak.sigma_x ** 2) +
                                             (y_current - peak.y0 - self.shift_xyz.shift_y()) ** 2 / (2 * peak.sigma_y ** 2))) *
                           np.exp(-((x_current - peak.x0 - self.shift_xyz.shift_x()) ** 2 / (2 * peak.sigma_x ** 2) +
                                    (y_current - peak.y0 - self.shift_xyz.shift_y()) ** 2 / (2 * peak.sigma_y ** 2))))
        #print(intensity)
        intensity -= self.target_isoline
        return intensity
