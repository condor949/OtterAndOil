import numpy as np
from collections.abc import Sequence
from vehicles import *
from .gnc import attitudeEuler
from tqdm import tqdm
from controllers import BaseController


def simultaneous_simulate(vehicles: Sequence, initial_positions: np.array, N: int, sample_time: float, controller: BaseController):
    DOF = 6  # degrees of freedom
    t = 0  # initial simulation time

    number_of_vehicles = len(vehicles)
    m_nu = []
    m_u_actual = []
    m_eta = []

    # Initial state vectors
    for k in range(number_of_vehicles):
        # position/attitude, user editable
        m_eta.append(np.array([initial_positions[k][1], initial_positions[k][0], 0, 0, 0, 0], float))

        # velocity, defined by vehicle class
        m_nu.append(vehicles[k].nu)

        # actual inputs, defined by vehicle class
        m_u_actual.append(vehicles[k].u_actual)

    # Initialization of table used to store the simulation data
    sim_data = [np.empty([N+1, 2 * DOF + 2 * vehicle.dimU], float) for vehicle in vehicles]

    # Simulator for-loop
    for i in tqdm(range(0, N + 1), desc="Vehicles Simulation"):

        m_u_control = controller.generate_control(vehicles, m_eta, i)

        for k in range(number_of_vehicles):
            eta = m_eta[k]
            nu = m_nu[k]
            u_actual = m_u_actual[k]
            u_control = m_u_control[k]

            t = i * sample_time  # simulation time

            # Store simulation data in simData
            signals = np.hstack((eta, nu, u_control, u_actual))
            sim_data[k][i, :] = signals

            # Propagate vehicle and attitude dynamics
            [nu, u_actual] = vehicles[k].dynamics(eta, nu, u_actual, u_control, sample_time)
            eta = attitudeEuler(eta, nu, sample_time)

            m_eta[k] = eta
            m_nu[k] = nu
            m_u_actual[k] = u_actual

    # Store simulation time vector
    sim_time = np.arange(start=0, stop=t + sample_time, step=sample_time)[:, None]

    return sim_time, sim_data
