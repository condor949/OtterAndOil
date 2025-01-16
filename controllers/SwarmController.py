from .BaseController import BaseController
import numpy as np


class SwarmController(BaseController):
    name = 'swarm'
    def __init__(self, starting_points, sample_time, optimizing_function, alpha1=0.1, alpha2=1):
        super().__init__()
        self.current_positions = np.array(starting_points)
        self.sample_time = sample_time
        self.optimizing_function = optimizing_function
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.personal_best_points = starting_points
        self.global_best_point = starting_points[np.argmax([optimizing_function(p) for p in starting_points])]

    def generate_control(self, vehicles, positions):
        velocities = (([p[:2] for p in positions] - self.current_positions) / self.sample_time)
        self.personal_best_points = np.array([
            p[:2] if self.optimizing_function(p[:2]) > self.optimizing_function(self.personal_best_points[i])
            else self.personal_best_points[i]
            for i, p in enumerate(positions)
        ])

        current_round_best_point = positions[np.argmax([self.optimizing_function(p) for p in positions])][:2]
        if self.optimizing_function(current_round_best_point) > self.optimizing_function(self.global_best_point):
            self.global_best_point = current_round_best_point

        desired_velocities = 0.05 *velocities +\
                        0.15 * (self.personal_best_points - [p[:2] for p in positions]) * np.random.random() +\
                        0.8 * np.array([self.global_best_point - p[:2] for p in positions]) * np.random.random()


        velocity_displaysment = desired_velocities - velocities

        radial = np.array([np.linalg.norm(v) for v in velocity_displaysment])
        angular = np.array([np.atan2(v[0], v[1]) for v in velocity_displaysment])

        control = [
            0.5*vehicles[i].n_max*np.tanh(radial[i])*np.array([1, 1]) + 0.1*vehicles[i].n_max*np.array([-1, 1])*np.sign(angular[i]) for  i in range(len(vehicles))
        ]

        return control




