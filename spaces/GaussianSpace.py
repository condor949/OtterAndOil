import numpy as np
from typing import Sequence
from spaces import BaseSpace, Peak
import matplotlib.pyplot as plt


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
        self.Z += self.shift_z

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
        # x01 = 0
        # y01 = 0
        # amplitude1 = 30
        # sigma_x1 = 6
        # sigma_y1 = 6
        # x02 = 5
        # y02 = 5
        # amplitude2 = 25
        # sigma_x2 = 1.5
        # sigma_y2 = 4
        # x03 = -5
        # y03 = -5
        # amplitude3 = 20
        # sigma_x3 = 3
        # sigma_y3 = 2
        # x04 = -3
        # y04 = 6
        # amplitude4 = 28
        # sigma_x4 = 1.5
        # sigma_y4 = 1.5
        # x05 = 4
        # y05 = -6
        # amplitude5 = 12
        # sigma_x5 = 1.8
        # sigma_y5 = 1
        # return (amplitude1 * np.exp(-((x_current - x01 - self.shift_x) ** 2 / (2 * sigma_x1 ** 2) +
        #                             (y_current - y01 - self.shift_y) ** 2 / (2 * sigma_y1 ** 2))) +
        #         amplitude2 * np.exp(-((x_current - x02 - self.shift_x) ** 2 / (2 * sigma_x2 ** 2) +
        #                             (y_current - y02 - self.shift_y) ** 2 / (2 * sigma_y2 ** 2))) +
        #         amplitude3 * np.exp(-((x_current - x03 - self.shift_x) ** 2 / (2 * sigma_x3 ** 2) +
        #                             (y_current - y03 - self.shift_y) ** 2 / (2 * sigma_y3 ** 2))) +
        #         amplitude4 * np.exp(-((x_current - x04 - self.shift_x) ** 2 / (2 * sigma_x4 ** 2) +
        #                             (y_current - y04 - self.shift_y) ** 2 / (2 * sigma_y4 ** 2))) +
        #         amplitude5 * np.exp(-((x_current - x05 - self.shift_x) ** 2 / (2 * sigma_x5 ** 2) +
        #                             (y_current - y05 - self.shift_y) ** 2 / (2 * sigma_y5 ** 2))) - 10)
        # idx_x = (np.abs(self.x - x_current)).argmin()
        # idx_y = (np.abs(self.y - y_current)).argmin()
        # print(self.Z[idx_x,idx_y])
        # return self.Z[idx_x, idx_y]
        # zed = griddata([x_current, y_current], self.Z, (self.X, self.Y), method='linear')
        # print(zed)
        # return zed[0][0][0]
        # return self.interp([x_current], [y_current])[0]

    def set_contour_points(self, plane_z=0, tol=1e-8):
        cont = []
        for i in range(len(self.X)):
            for j in range(len(self.X[i])):
                if abs(self.Z[i, j] - plane_z) < tol:
                    cont.append((self.X[i, j], self.Y[i, j]))
        self.contour_points = cont

    def get_nearest_contour_point_norm(self, x, y):
        return min([(xc-x)**2+(yc-y)**2 for xc, yc in self.contour_points])**0.5

    def plot_surface(self, path, title="3D Gaussian Surface"):
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
        plt.savefig(path)
        plt.close()