import numpy as np
from collections.abc import Sequence
from vehicles import *
from .gnc import attitudeEuler
from tqdm import tqdm
from controllers import BaseController


def simultaneous_simulate(vehicles: Sequence, N: int, sample_time: float, controller: BaseController):
    DOF = 6  # degrees of freedom
    t = 0  # initial simulation time

    m_nu = []
    m_u_actual = []
    m_eta = []

    # Initial state vectors
    for vehicle in vehicles:
        # position/attitude, user editable
        m_eta.append(np.array([vehicle.starting_point[1], vehicle.starting_point[0], 0, 0, 0, 0], float))

        # velocity, defined by vehicle class
        m_nu.append(vehicle.nu)

        # actual inputs, defined by vehicle class
        m_u_actual.append(vehicle.u_actual)

    # Initialization of table used to store the simulation data
    sim_data = [np.empty([N+1, 2 * DOF + 2 * vehicle.dimU], float) for vehicle in vehicles]

    # Simulator for-loop
    for i in tqdm(range(0, N + 1), desc=f"Vehicle Simulation x{len(vehicles)}"):

        m_u_control = controller.generate_control(vehicles, m_eta, i)

        for vehicle in vehicles:
            eta = m_eta[vehicle.serial_number]
            nu = m_nu[vehicle.serial_number]
            u_actual = m_u_actual[vehicle.serial_number]
            u_control = m_u_control[vehicle.serial_number]

            t = i * sample_time  # simulation time

            # Store simulation data in simData
            signals = np.hstack((eta, nu, u_control, u_actual))
            sim_data[vehicle.serial_number][i, :] = signals

            # Propagate vehicle and attitude dynamics
            [nu, u_actual] = vehicle.dynamics(eta, nu, u_actual, u_control, sample_time)
            eta = attitudeEuler(eta, nu, sample_time)

            m_eta[vehicle.serial_number] = eta
            m_nu[vehicle.serial_number] = nu
            m_u_actual[vehicle.serial_number] = u_actual

    # Store simulation time vector
    controller.set_sim_time(np.arange(start=0, stop=t + sample_time, step=sample_time)[:, None])

    return sim_data
