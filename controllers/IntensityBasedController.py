import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import pi
from tqdm import tqdm
from matplotlib import rc, rcParams, rcParamsDefault
rcParams.update(rcParamsDefault)
rcParams["text.usetex"] = True
from spaces import BaseSpace
from functools import partial
from tools.dataStorage import *
from numpy.ma.core import cumsum
from .BaseController import BaseController
from tools.random_generators import normalize
from tools.random_generators import color_generator

legendSize = 16
legendSize1 = 10
figSize1 = [25, 25]  # figure1 size in cm
bigFigSize1 = [50, 26]  # larger figure1 size in cm
figSize2 = [25, 13]  # figure2 size in cm
dpiValue = 150  # figure dpi value

def R2D(value):  # radians to degrees
    return value * 180 / pi

def cm2inch(value):  # inch to cm
    return value / 2.54


class IntensityBasedController(BaseController):
    name = 'intensity'
    def __init__(self, vehicles, sim_time: int, sample_time: float, space: BaseSpace,
                 eps: float,
                 e_max_cap: float,
                 dynamic_error_max: bool,
                 smoothing: float,
                 plot_config_path: str,
                 FPS=30,
                 isolines=10,
                 mu=1):
        super().__init__(vehicles=vehicles,
                         sim_time=sim_time,
                         sample_time=sample_time,
                         space=space,
                         eps=eps,
                         e_max_cap=e_max_cap,
                         dynamic_error_max=dynamic_error_max,
                         smoothing=smoothing,
                         plot_config_path=plot_config_path)
        self.m_f_prev = [space.get_intensity(vehicle.starting_point[1], vehicle.starting_point[0]) for vehicle in vehicles]
        self.mu = mu
        self.FPS = FPS
        self.isolines = isolines
        self.intensity = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.der = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.mu_tanh = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.sigmas = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.quality_array = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        self.type = 'Individual intensity based controller'
        self.times_outside = []
        self.u_controls = []
        self.dss = []
        self.n_rots = []
        self.n_forwards = []
        self.nus = []

    def __str__(self):
        return (f'---controller--------------------------------------------------------------------------\n'
                f'{self.type}\n'
                f'Sampling frequency: {round(1 / self.sample_time)} Hz\n'
                f'Sampling time: {self.sample_time} seconds\n'
                f'Simulation time: {round(self.sim_time)} seconds\n'
                f'Numbers of vehicles: {self.number_of_vehicles}')

    def matveev_law(self, vehicle, step, f_current, f_prev):
        self.der[vehicle, step] = (f_current - f_prev) / self.sample_time
        self.mu_tanh[vehicle, step] = self.mu * np.tanh(f_current) #- self.f_target)
        self.sigmas[vehicle, step] = -np.sign(self.der[vehicle, step] + self.mu_tanh[vehicle, step])
        #self.sigmas[vehicle, step] = -np.sign(self.der[vehicle, step] + ds * self.mu_tanh[vehicle, step])
        return self.sigmas[vehicle, step]

    def generate_control(self, positions, step, relative_velocities):
        m_f_current = [self.space.get_intensity(eta[1], eta[0]) for eta in positions]
        controls = []
        for vehicle in self.vehicles:
            f_current = m_f_current[vehicle.serial_number]
            #print(f_current)
            f_prev = self.m_f_prev[vehicle.serial_number]
            nu = relative_velocities[vehicle.serial_number]
            self.nus.append(nu)
            sigma = self.matveev_law(vehicle.serial_number, step, f_current, f_prev)
            self.errors[vehicle.serial_number,step] = abs(f_current)
            e_norm = self.update_error_metrics(f_current)
            self.sum_error_values[vehicle.serial_number] += e_norm
            self.errors_max[vehicle.serial_number, step] = self.e_max
            self.errors_norm[vehicle.serial_number, step] = e_norm
            self.errors_avg[vehicle.serial_number] = self.moving_average(self.errors_norm[vehicle.serial_number], 1000)
            if sigma < 0:
                u_control = [vehicle.n_min, vehicle.n_max]
            elif sigma > 0:
                u_control = [vehicle.n_max, vehicle.n_min]
            else:
                u_control = [0, 0]
            controls.append(u_control)
            self.intensity[vehicle.serial_number, step] = f_current
            self.quality_array[vehicle.serial_number, step] = self.space.get_nearest_contour_point_norm(positions[vehicle.serial_number][0], positions[vehicle.serial_number][1])

        self.m_f_prev = m_f_current
        return controls

    def plotting_sigma(self, store_plot=False, **arguments):
        # print(np.array(self.der).shape)
        # print(np.array(self.mu_tanh).shape)
        # print(np.array(self.sigmas).shape)
        result = np.array([normalize(der_vehicle, 10) for der_vehicle in self.der])

        for vehicle in range(self.number_of_vehicles):
            plt.figure()
            rc('font', size=30)
            result[vehicle][0] = 0
            plt.plot(self.simTime, result[vehicle], label='$\\dot{f}$')
            plt.plot(self.simTime, self.mu_tanh[vehicle], label='$\\mu\\chi(f - f_\\text{target})$')
            plt.hlines(0, xmin=0, xmax=np.max(self.simTime), color='gray') # 0 axis
            plt.xlabel('Time, s', fontsize=12)
            # plt.ylabel('Sigma', fontsize=12)
            plt.legend()
            if store_plot:
                plt.tight_layout()
                plt.savefig(self.data_storage.get_path(f'sigmas_v{vehicle}','png'))
                plt.close()
            else:
                plt.title(f"Control v{vehicle}", fontsize=10)
                plt.tight_layout()
                plt.show()

    def plotting_cumulative(self, store_plot=False, **arguments):
        cumulative_quality = cumsum(self.sample_time*self.quality_array, axis=1)

        plt.figure()
        rc('font', size=30)
        plt.xlabel('Time, s')
        plt.ylabel(ylabel='Total area integral, $m^2$')
        for i in range(self.number_of_vehicles):
            plt.plot(self.simTime, cumulative_quality[i], label=f'Agent {i+1}', color=self.colors.get(i))
        plt.legend()
        if store_plot:
            plt.tight_layout()
            plt.savefig(self.data_storage.get_path('cumulative', 'png'))
            plt.close()
        else:
            plt.title('The total value of the shortest distance at each point of the trajectory',
                      fontsize=10)
            plt.tight_layout()
            plt.show()
        cumulative_quality.dump('cumulative.npy')

    def plotting_short_distance(self, store_plot=False, **arguments):
        plt.figure()
        plt.xlabel('Time, s', fontsize=12)
        plt.ylabel('Short distance, m', fontsize=12)
        plt.title('The total value of the shortest distance at each point of the trajectory',
                  fontsize=10)
        #print(np.array(self.simTime).shape)
        #print(np.array(self.quality_array).shape)
        rc('font', size=30)
        for i in range(self.number_of_vehicles):
            plt.plot(self.simTime, self.quality_array[0], label=f'Agent {i+1}', color=self.colors.get(i))
        plt.legend()
        if store_plot:
            plt.tight_layout()
            plt.savefig(self.data_storage.get_path('short_distance', 'png'))
            plt.close()
        else:
            plt.tight_layout()
            plt.show()
        self.quality_array.dump('short_distance.npy')

    def plotting_track(self, swarmData=None,
                           big_picture: bool = False,
                           not_animated: bool = False,
                           store_plot: bool = False,
                           **arguments):
        # Attaching 3D axis to the figure
        if big_picture:
            fig = plt.figure(figsize=(cm2inch(bigFigSize1[0]), cm2inch(bigFigSize1[1])),
                             dpi=dpiValue)
        else:
            fig = plt.figure(figsize=(cm2inch(figSize1[0]), cm2inch(figSize1[1])),
                             dpi=dpiValue)

        rc('font', size=30)
        contour = plt.contour(self.space.get_X(), self.space.get_Y(), self.space.get_Z(), levels=self.isolines,
                              cmap='viridis')  # Adjust `levels` to set number of isolines

        plt.clabel(contour, inline=True)  # Add labels to the isolines
        plt.xlabel('X, m / East')
        plt.ylabel('Y, m / North')
        # plt.colorbar(contour, label='Intensity')  # Add a color bar for reference
        plt.contour(self.space.get_X(), self.space.get_Y(), self.space.get_Z(), levels=[self.space.target_isoline],
                    colors='red')  # Intersection line

        plotData = {}
        quivers = []

        def anim_function(num, plotData, quivers):
            for i, (line, dataSet) in enumerate(plotData.items()):
                line.set_data(dataSet[0:2, :num])

                # обновляем положение лодки (стрелочки)
                if num < dataSet.shape[1] - 1:
                    dx = dataSet[0, num + 1] - dataSet[0, num]
                    dy = dataSet[1, num + 1] - dataSet[1, num]
                else:  # если последняя точка, берем предыдущий шаг
                    dx = dataSet[0, num] - dataSet[0, num - 1]
                    dy = dataSet[1, num] - dataSet[1, num - 1]

                norm = np.hypot(dx, dy)
                if norm < 1e-6:
                    dx, dy = 1.0, 0.0  # или dx, dy = 0, 0 чтобы не было стрелки вообще
                else:
                    dx, dy = dx / norm, dy / norm  # нормализуем
                    arrow_length = 2.0  # выберите подходящее значение
                    dx *= arrow_length
                    dy *= arrow_length

                quivers[i].set_offsets([dataSet[0, num], dataSet[1, num]])
                quivers[i].set_UVC(dx, dy)

            return list(plotData.keys()) + quivers

        color_gen = color_generator()
        i = 0
        for simData in swarmData:
            # State vectors
            x = simData[:, 0]
            y = simData[:, 1]
            z = simData[:, 2]

            N = y[::len(x) // self.space.grid_size]
            E = x[::len(x) // self.space.grid_size]
            D = z[::len(x) // self.space.grid_size]

            dataSet = np.array([N, E, -D])  # Down is negative z

            color = next(color_gen)
            start_x = dataSet[0][0]
            start_y = dataSet[1][0]
            initial_dx = dataSet[0][1] - dataSet[0][0]
            initial_dy = dataSet[1][1] - dataSet[1][0]
            plt.plot(start_x, start_y, marker='*', markersize=10, color=color, label='_nolegend_',
                     zorder=10)

            # plt.text(start_x, start_y, f'({start_x:.1f}, {start_y:.1f})',
            #          fontsize=12, color=color, verticalalignment='bottom', horizontalalignment='right')
            quiv = plt.quiver(
                start_x, start_y, initial_dx, initial_dy,
                angles='xy',
                scale_units='xy',
                scale=1,
                color=color,
                width=0.02, zorder=15, label='_nolegend_'
            )
            quivers.append(quiv)

            line = plt.plot(dataSet[0], dataSet[1], lw=2, c=color, zorder=10, label=f'agent {i + 1}')[0]
            plotData[line] = dataSet
            i += 1

        plt.legend()

        if store_plot:
            plt.tight_layout()
            plt.savefig(self.data_storage.get_path('track', "png"))
        else:
            plt.tight_layout()
            not_animated = True
            plt.title('Track in the intensity field')
            plt.show()

        if not not_animated:
            # Create the animation object
            ani = animation.FuncAnimation(fig,
                                          partial(anim_function, plotData=plotData, quivers=quivers),
                                          frames=self.space.grid_size,
                                          interval=200,
                                          blit=False,
                                          repeat=True)

            update_func = lambda _i, _n: progress_bar.update(1)
            with tqdm(total=getattr(ani, "_save_count"), desc="Animation Writing") as progress_bar:
                try:
                    writer = animation.FFMpegWriter(fps=self.FPS)
                except:
                    writer = animation.PillowWriter(fps=self.FPS)

                ani.save(self.data_storage.get_path('track', "gif"),
                         writer=writer,
                         progress_callback=update_func)

        plt.close()