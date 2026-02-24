import logging
from typing import Dict

import numpy as np

from controllers.BaseController import BaseController

logger = logging.getLogger(__name__)


class StepInputController(BaseController):
    """
    Controller that uses stepInput method for each vehicle.
    
    The stepInput generates propeller step inputs based on time.
    """
    name = 'step_input'
    abbreviation = 'SI'
    controller_type = 'individual'
    
    def __init__(self, vehicles, sim_time: int, sample_time: float, space,
                 eps: float, e_max_cap: float, dynamic_error_max: bool,
                 smoothing: float, FPS=30, **kwargs):
        """
        Initialize the step input controller.
        
        Args:
            vehicles: List of vehicle instances
            sim_time: Simulation time in seconds
            sample_time: Sample time in seconds
            space: Space instance (not used by this controller, but required by interface)
            eps: Error threshold (not used, but required by interface)
            e_max_cap: Maximum error cap (not used, but required by interface)
            dynamic_error_max: Dynamic error max flag (not used, but required by interface)
            smoothing: Smoothing factor (not used, but required by interface)
            FPS: Frames per second for visualization
        """
        super().__init__(vehicles, sim_time, sample_time, space, eps, e_max_cap,
                        dynamic_error_max, smoothing)
        
        self.FPS = FPS
        
        # Атрибуты для визуализации (даже если не используются для управления)
        self.intensity = np.zeros((self.number_of_vehicles, self.N), dtype=float)
        
        # Проверка наличия метода stepInput для каждого транспортного средства
        for vehicle in self.vehicles:
            if hasattr(vehicle, 'stepInput'):
                logger.debug(
                    "Configured step input controller for vehicle %d",
                    vehicle.serial_number
                )
            else:
                logger.warning(
                    "Vehicle %s does not support stepInput method",
                    vehicle
                )
    
    def generate_control(self, positions, step, relative_velocities):
        """
        Генерирует управляющие сигналы используя stepInput для каждого транспортного средства.
        
        Args:
            positions: List of position vectors (eta) for each vehicle
            step: Current simulation step
            relative_velocities: List of velocity vectors (nu) for each vehicle
            
        Returns:
            List of control input vectors [n1, n2] for each vehicle
        """
        controls = []
        
        # Вычисляем текущее время
        t = step * self.sample_time
        
        # Отслеживаем интенсивность для визуализации (даже если не используем для управления)
        if self.space is not None:
            m_f_current = [self.space.get_intensity(eta[1], eta[0]) for eta in positions]
            for i, f_current in enumerate(m_f_current):
                if step < self.N:
                    self.intensity[i, step] = f_current
                    # Также обновляем ошибки для визуализации
                    e = abs(f_current)
                    e_norm = self.update_error_metrics(f_current)
                    self.sum_error_values[i] += e_norm
                    if step < self.N:
                        self.errors_max[i, step] = self.e_max
                        self.errors_norm[i, step] = e_norm
        
        for i, vehicle in enumerate(self.vehicles):
            if hasattr(vehicle, 'stepInput'):
                # Используем stepInput для управления
                u_control = vehicle.stepInput(t)
            else:
                # Fallback: нулевое управление
                logger.warning(
                    "Vehicle %s does not support stepInput, using zero control",
                    vehicle
                )
                u_control = np.array([0, 0], float)
            
            controls.append(u_control)
        
        return controls
    
    def track_snapshot(self) -> Dict[str, object]:
        """Return metadata required to render simple track plots (without intensity field)."""
        colors = [self.colors[i] for i in range(self.number_of_vehicles)] if self.colors else None
        serial_numbers = [vehicle.serial_number for vehicle in self.vehicles]
        return {
            "simple_track": True,  # Flag to use simple track renderer
            "grid_size": 50,  # For animation frames
            "fps": self.FPS,
            "colors": colors,
            "serial_numbers": serial_numbers,
        }
    
    def __str__(self):
        return (f'---controller--------------------------------------------------------------------------\n'
                f'Step Input Controller\n'
                f'Sampling frequency: {round(1 / self.sample_time)} Hz\n'
                f'Sampling time: {self.sample_time} seconds\n'
                f'Simulation time: {round(self.sim_time)} seconds\n'
                f'Numbers of vehicles: {self.number_of_vehicles}')
