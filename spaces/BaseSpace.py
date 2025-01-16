import numpy as np
import matplotlib.pyplot as plt

from abc import ABC
from tools.dataStorage import *
from collections.abc import Sequence


class Peak:
    def __init__(self, x0, y0, amplitude, sigma_x, sigma_y):
        self.x0 = x0
        self.y0 = y0
        self.amplitude = amplitude
        self.sigma_x = sigma_x
        self.sigma_y = sigma_y

    def __repr__(self):
        return (f"Peak(x0={self.x0}, y0={self.y0}, amplitude={self.amplitude}, "
                f"sigma_x={self.sigma_x}, sigma_y={self.sigma_y})")

    def get_json_data(self):
        return {
            "x0": self.x0,
            "y0": self.y0,
            "amplitude": self.amplitude,
            "sigma_x": self.sigma_x,
            "sigma_y": self.sigma_y
        }


class ShiftingSpace:
    def __init__(self, shift_xyz=None):
        if shift_xyz is None:
            self.shift_xyz = [0, 0, 0]
        else:
            self.shift_xyz = shift_xyz

    def __repr__(self):
        return self.shift_xyz

    def __str__(self):
        return (f"\n\tshift_x={self.shift_x()}\n"
                f"\tshift_y={self.shift_y()}\n"
                f"\tshift_z={self.shift_z()}")

    def shift_x(self):
        return self.shift_xyz[0]

    def shift_y(self):
        return self.shift_xyz[1]

    def shift_z(self):
        return self.shift_xyz[2]


class BaseSpace(ABC):
    name = 'base_space'

    def __init__(self, x_range=(-30, 30), y_range=(-30, 30), grid_size=500, shift_xyz=None, space_filename="",
                 target_isoline=0):
        """
        Initialize the 3D space.

        Parameters:
        x_range (tuple): Range of x-axis values.
        y_range (tuple): Range of y-axis values.
        grid_size (int): Number of points in each dimension.
        shift_xyz (int): Shift of all points of space by values from this array, respectively XYZ.
        """
        self.x = np.linspace(*x_range, int(grid_size))
        self.y = np.linspace(*y_range, grid_size)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        self.Z = np.zeros_like(self.X)  # Start with a flat surface
        self.shift_xyz = ShiftingSpace(shift_xyz)
        self.interp = None
        self.contour_points = list()
        self.peaks = list()
        if space_filename:
            with open(space_filename, 'r') as file:
                data = json.load(file)
                self.peaks = [Peak(**item) for item in data]
        self.type = ""
        self.target_isoline = target_isoline
        self.grid_size = grid_size
        self.data_storage = None

    def __str__(self):
        return (f'---space--------------------------------------------------------------------\n'
                f'Space type: {self.type}\n'
                f'Shifting space: {self.shift_xyz}\n'
                f'Target isoline: {self.target_isoline}')

    def set_contour_points(self, plane_z=0, tol=1e-8):
        cont = []
        for i in range(len(self.X)):
            for j in range(len(self.X[i])):
                if abs(self.Z[i, j] - plane_z) < tol:
                    cont.append((self.X[i, j], self.Y[i, j]))
        self.contour_points = cont

    def get_nearest_contour_point_norm(self, x, y):
        return min([(xc - x) ** 2 + (yc - y) ** 2 for xc, yc in self.contour_points]) ** 0.5

    def get_intensity(self, x_current, y_current):
        pass

    def plotting_surface(self, store_plot=False, **arguments):
        """
        Plot the 3D surface with all peaks.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(self.X, self.Y, self.Z, cmap='viridis')
        ax.set_xlabel('X,m / East')
        ax.set_ylabel('Y,m / North')
        ax.set_zlabel('Intensity')
        if store_plot:
            plt.savefig(self.data_storage.get_path(self.name, 'png'))
            plt.close()
        else:
            plt.title(f"Intensity map, based on {self.name} peaks")
            plt.show()

    def get_json_data(self):
        json_data = list()
        for peak in self.peaks:
            json_data.append(peak.get_json_data())
        return json_data

    def store_in_config(self):
        with open(self.data_storage.get_path("space", "json"), 'w') as config:
            json.dump(self.get_json_data(), config, indent=4)

    def get_X(self) -> Sequence:
        return self.X

    def get_Y(self) -> Sequence:
        return self.Y

    def get_Z(self) -> Sequence:
        return self.Z

    def set_data_storage(self, data_storage):
        self.data_storage = data_storage
