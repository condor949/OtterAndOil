#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3dotter.py: 
    3-DOF horizontal-plane reduction of otter.py
    
    Class for the Maritime Robotics Otter USV in 3-DOF (surge, sway, yaw).
    The length of the USV is L = 2.0 m. The constructors are:

    Otter3D()                                          
        Step inputs for propeller revolutions n1 and n2
        
Methods:
    
[nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) returns 
    nu[k+1] and u_actual[k+1] using Euler's method. The control inputs are:

    u_control = [ n1 n2 ]' where 
        n1: propeller shaft speed, left (rad/s)
        n2: propeller shaft speed, right (rad/s)

u = stepInput(t) generates propeller step inputs.

[n1, n2] = controlAllocation(tau_X, tau_N)     
    Control allocation algorithm.
    
References: 
  T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
     Control. 2nd. Edition, Wiley. 
     URL: www.fossen.biz/wiley            

Author:     Derived from otter.py by Thor I. Fossen
"""
import math
import numpy as np
from .vehicle import *
from lib.gnc import Smtrx, Hmtrx, Rzyx, m2c, crossFlowDrag, sat
from tools.diagnostic_logger import DiagnosticLogger


# Class Vehicle
class Otter3D(Vehicle):
    """
    Otter3D()                                           Propeller step inputs
    """
    name = 'otter3d'
    def __init__(
            self,
            controlSystem="stepInput",
            V_current=0,
            beta_current=0,
            serial_number=0,
            shift=None,
            color='b',
            starting_point=None
    ):
        super().__init__(V_current,
                         serial_number,
                         shift,
                         color,
                         starting_point)
        # Constants
        D2R = math.pi / 180  # deg2rad
        self.g = 9.81  # acceleration of gravity (m/s^2)
        rho = 1026  # density of water (kg/m^3)

        # Set current speed and direction
        self.V_c = V_current
        self.beta_c = beta_current * D2R

        # Control system description
        if controlSystem == "stepInput":
            self.controlDescription = "Step inputs for n1 and n2"
        elif controlSystem == "customTestInput":
            self.controlDescription = "Custom test sequence: 0-30s off, 30-60s full ahead, 60-90s asymmetric, 90-100s off"
        else:
            self.controlDescription = "sigma"
            controlSystem = "Berman Law"

        self.controlMode = controlSystem

        # Initialize the Otter USV model
        self.T_n = 1.0  # propeller time constants (s)
        self.L = 2.0  # Length (m)
        self.B = 1.08  # beam (m)
        self.nu = np.array([0, 0, 0], float)  # velocity vector [u, v, r]
        self.u_actual = np.array([0, 0], float)  # propeller revolution states
        self.type = "Otter USV 3-DOF (see 'otter_3d.py' for more details)"

        self.controls = [
            "Left propeller shaft speed (rad/s)",
            "Right propeller shaft speed (rad/s)"
        ]
        self.dimU = len(self.controls)

        # Vehicle parameters (same as otter.py)
        m = 55.0  # mass (kg)
        self.mp = 25.0  # Payload (kg)
        self.m_total = m + self.mp
        self.rp = np.array([0.05, 0, -0.35], float)  # location of payload (m)
        rg = np.array([0.2, 0, -0.2], float)  # CG for hull only (m)
        rg = (m * rg + self.mp * self.rp) / (m + self.mp)  # CG corrected for payload
        self.S_rg = Smtrx(rg)
        self.H_rg = Hmtrx(rg)
        self.S_rp = Smtrx(self.rp)

        R44 = 0.4 * self.B  # radii of gyration (m)
        R55 = 0.25 * self.L
        R66 = 0.25 * self.L
        T_sway = 1.0        # time constant in sway (s)
        T_yaw = 1.0  # time constant in yaw (s)
        Umax = 6 * 0.5144  # max forward speed (m/s)

        # Data for one pontoon
        self.B_pont = 0.25  # beam of one pontoon (m)
        y_pont = 0.395  # distance from centerline to waterline centroid (m)
        Cw_pont = 0.75  # waterline area coefficient (-)
        Cb_pont = 0.4  # block coefficient, computed from m = 55 kg

        # Inertia dyadic, volume displacement and draft
        nabla = (m + self.mp) / rho  # volume
        self.T = nabla / (2 * Cb_pont * self.B_pont * self.L)  # draft
        Ig_CG = m * np.diag(np.array([R44 ** 2, R55 ** 2, R66 ** 2]))
        self.Ig = Ig_CG - m * self.S_rg @ self.S_rg - self.mp * self.S_rp @ self.S_rp

        # Experimental propeller data including lever arms
        self.l1 = -y_pont  # lever arm, left propeller (m)
        self.l2 = y_pont  # lever arm, right propeller (m)
        self.k_pos = 0.02216 / 2  # Positive Bollard, one propeller
        self.k_neg = 0.01289 / 2  # Negative Bollard, one propeller
        self.n_max = math.sqrt((0.5 * 24.4 * self.g) / self.k_pos)  # max. prop. rev.
        self.n_min = -math.sqrt((0.5 * 13.6 * self.g) / self.k_neg)  # min. prop. rev.

        # Build 6-DOF MRB exactly like otter.py
        # MRB_CG = [ (m+mp) * I3  O3      (Fossen 2021, Chapter 3)
        #               O3       Ig ]
        MRB_CG = np.zeros((6, 6))
        MRB_CG[0:3, 0:3] = (m + self.mp) * np.identity(3)
        MRB_CG[3:6, 3:6] = self.Ig
        MRB_6 = self.H_rg.T @ MRB_CG @ self.H_rg

        # Hydrodynamic added mass (best practice) - same as otter.py
        Xudot = -0.1 * m
        Yvdot = -1.5 * m
        Zwdot = -1.0 * m
        Kpdot = -0.2 * self.Ig[0, 0]
        Mqdot = -0.8 * self.Ig[1, 1]
        Nrdot = -1.7 * self.Ig[2, 2]

        self.MA_6 = -np.diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot])

        # System mass matrix in 6-DOF
        M_6 = MRB_6 + self.MA_6

        # Reduce to 3-DOF by extracting indices [0, 1, 5] (u, v, r)
        self.M = M_6[np.ix_([0, 1, 5], [0, 1, 5])]
        self.Minv = np.linalg.inv(self.M)

        # Hydrostatic quantities (Fossen 2021, Chapter 4) - computed but not used in 3-DOF
        Aw_pont = Cw_pont * self.L * self.B_pont  # waterline area, one pontoon
        I_T = (
                2
                * (1 / 12)
                * self.L
                * self.B_pont ** 3
                * (6 * Cw_pont ** 3 / ((1 + Cw_pont) * (1 + 2 * Cw_pont)))
                + 2 * Aw_pont * y_pont ** 2
        )
        I_L = 0.8 * 2 * (1 / 12) * self.B_pont * self.L ** 3
        KB = (1 / 3) * (5 * self.T / 2 - 0.5 * nabla / (self.L * self.B_pont))
        BM_T = I_T / nabla  # BM values
        BM_L = I_L / nabla
        KM_T = KB + BM_T  # KM values
        KM_L = KB + BM_L
        KG = self.T - rg[2]
        GM_T = KM_T - KG  # GM values
        GM_L = KM_L - KG

        G33 = rho * self.g * (2 * Aw_pont)  # spring stiffness
        G44 = rho * self.g * nabla * GM_T
        G55 = rho * self.g * nabla * GM_L
        G_CF = np.diag([0, 0, G33, G44, G55, 0])  # spring stiff. matrix in CF
        LCF = -0.2
        H = Hmtrx(np.array([LCF, 0.0, 0.0]))  # transform G_CF from CF to CO
        self.G_6 = H.T @ G_CF @ H
        # In 3-DOF horizontal model, G_3D = 0 (no hydrostatic restoring in horizontal plane)
        self.G = np.zeros((3, 3))

        # Natural frequencies (for damping computation)
        w3 = math.sqrt(G33 / M_6[2, 2])
        w4 = math.sqrt(G44 / M_6[3, 3])
        w5 = math.sqrt(G55 / M_6[4, 4])

        # Linear damping terms (hydrodynamic derivatives) - same as otter.py
        Xu = -24.4 * self.g / Umax  # specified using the maximum speed
        Yv = -self.M[1, 1]  / T_sway # specified using the time constant in sway
        Zw = -2 * 0.3 * w3 * M_6[2, 2]  # specified using relative damping
        Kp = -2 * 0.2 * w4 * M_6[3, 3]
        Mq = -2 * 0.4 * w5 * M_6[4, 4]
        Nr = -M_6[5, 5] / T_yaw  # specified by the time constant T_yaw

        self.D_6 = -np.diag([Xu, Yv, Zw, Kp, Mq, Nr])
        # Reduce to 3-DOF
        self.D = self.D_6[np.ix_([0, 1, 5], [0, 1, 5])]

        # Propeller configuration/input matrix
        B = self.k_pos * np.array([[1, 1], [-self.l1, -self.l2]])
        self.Binv = np.linalg.inv(B)
        
        # Diagnostic logger (will be set externally if needed)
        self.diagnostic_logger = None
        self.step_count = 0

    def __str__(self):
        if self.starting_point is not None:
            sp_str = f'[{self.starting_point[0]}, {self.starting_point[1]}]'
        else:
            sp_str = '[None]'
        return (f'---vehicle--------------------------------------------------------------------------\n'
                f'{self.type}\n'
                f'Length: {self.L} m\n'
                f'Control: {self.controlDescription}\n'
                f'Starting point: {sp_str}')

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        """
        [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates
        the Otter USV 3-DOF equations of motion using Euler's method.
        """

        # Input vector
        n = np.array([u_actual[0], u_actual[1]])

        # Current velocities - same computation as otter.py
        psi = eta[2]  # yaw angle in 3-DOF
        r = nu[2]  # yaw rate
        u_c = self.V_c * math.cos(self.beta_c - psi)  # current surge vel.
        v_c = self.V_c * math.sin(self.beta_c - psi)  # current sway vel.


        # Build 6-DOF current vectors (for consistency with otter.py)
        nu_c_6 = np.array([u_c, v_c, 0, 0, 0, 0], float)  # current velocity vector
        Dnu_c_6 = np.array([r * v_c, -r * u_c, 0, 0, 0, 0], float)  # derivative
        
        # Reduce to 3-DOF
        nu_c_3 = nu_c_6[[0, 1, 5]]  # [u_c, v_c, 0]
        Dnu_c_3 = Dnu_c_6[[0, 1, 5]]  # [r*v_c, -r*u_c, 0]

        # Build 6-DOF nu for matrix computations
        nu_6 = np.array([nu[0], nu[1], 0, 0, 0, nu[2]], float)  # [u, v, 0, 0, 0, r]
        nu_r_6 = nu_6 - nu_c_6  # relative velocity vector in 6-DOF
        nu_r_3 = nu_r_6[[0, 1, 5]]  # [u_r, v_r, r_r]

        # Rigid body and added mass Coriolis and centripetal matrices
        # Compute CRB in 6-DOF exactly like otter.py
        # CRB_CG = [ (m+mp) * Smtrx(nu2)          O3   (Fossen 2021, Chapter 6)
        #              O3                   -Smtrx(Ig*nu2)  ]
        CRB_CG = np.zeros((6, 6))
        CRB_CG[0:3, 0:3] = self.m_total * Smtrx(nu_6[3:6])
        CRB_CG[3:6, 3:6] = -Smtrx(np.matmul(self.Ig, nu_6[3:6]))
        CRB_6 = self.H_rg.T @ CRB_CG @ self.H_rg  # transform CRB from CG to CO

        # Compute CA in 6-DOF exactly like otter.py
        CA_6 = m2c(self.MA_6, nu_r_6)
       # CA_6[5, 0] = 0  # assume that the Munk moment in yaw can be neglected
       # CA_6[5, 1] = 0  # if nonzero, must be balanced by adding nonlinear damping
       # CA_6[0, 5] = 0
       # CA_6[1, 5] = 0

        C_6 = CRB_6 + CA_6
        # Reduce to 3-DOF
        C_3 = C_6[np.ix_([0, 1, 5], [0, 1, 5])]

        # Payload force and moment - computed but not used in 3-DOF horizontal model
        # (g_0 is mainly z/roll/pitch related, so omit in 3-DOF)
        g_0_3 = np.array([0, 0, 0], float)

        # Control forces and moments - with propeller revolution saturation
        thrust = np.zeros(2)
        for i in range(0, 2):

            n[i] = sat(n[i], self.n_min, self.n_max)  # saturation, physical limits

            if n[i] > 0:  # positive thrust
                thrust[i] = self.k_pos * n[i] * abs(n[i])
            else:  # negative thrust
                thrust[i] = self.k_neg * n[i] * abs(n[i])

        # Control forces and moments - same as otter.py
        tau_X = thrust[0] + thrust[1]
        tau_N = -self.l1 * thrust[0] - self.l2 * thrust[1]
        tau_3 = np.array([tau_X, 0, tau_N], float)

        # Hydrodynamic linear damping + nonlinear yaw damping
        # Compute in 6-DOF first (for consistency with otter.py)
        tau_damp_6 = -np.matmul(self.D_6, nu_r_6)
        tau_damp_6[5] = tau_damp_6[5] - 10 * self.D_6[5, 5] * abs(nu_r_6[5]) * nu_r_6[5]
        
        # Reduce to 3-DOF
        tau_damp_3 = tau_damp_6[[0, 1, 5]]

        # Cross-flow drag - compute in 6-DOF, then reduce
        tau_crossflow_6 = crossFlowDrag(self.L, self.B_pont, self.T, nu_r_6)
        tau_crossflow_3 = tau_crossflow_6[[0, 1, 5]]

        # State derivatives (with dimension)
        sum_tau_3 = (
                tau_3
                + tau_damp_3
                + tau_crossflow_3
                - np.matmul(C_3, nu_r_3)
                + g_0_3  # g_0_3 = 0, but keep for consistency
        )

        nu_dot_3 = Dnu_c_3 + np.matmul(self.Minv, sum_tau_3)  # USV dynamics
        n_dot = (u_control - n) / self.T_n  # propeller dynamics

        real = False
        # Forward Euler integration [k+1]
        if real:
            n = u_control
        else:
            n = n + sampleTime * n_dot
        nu = nu + sampleTime * nu_dot_3

        u_actual = np.array(n, float)

        # Diagnostic logging
        if self.diagnostic_logger is not None:
            # Compute eta_dot for logging (matching repositioning formulas)
            psi = eta[2]
            u = nu[0]
            v = nu[1]
            r = nu[2]
            # Match Rzyx convention: y_dot = cos(psi)*u - sin(psi)*v, x_dot = sin(psi)*u + cos(psi)*v
            y_dot = math.cos(psi) * u - math.sin(psi) * v
            x_dot = math.sin(psi) * u + math.cos(psi) * v
            psi_dot = r
            eta_dot_3dof = np.array([y_dot, x_dot, psi_dot], float)  # y_dot, x_dot, psi_dot
            
            self.diagnostic_logger.log_step(
                t=self.step_count * sampleTime,
                n_cmd=u_control,
                n_actual=u_actual,
                thrust=thrust,
                tau=tau_3,
                nu=nu,
                eta=eta,
                nu_dot=nu_dot_3,
                eta_dot=eta_dot_3dof,
                nu_c=nu_c_3,
                V_c=self.V_c,
                beta_c=self.beta_c
            )
            self.step_count += 1

        return nu, u_actual

    def controlAllocation(self, tau_X, tau_N):
        """
        [n1, n2] = controlAllocation(tau_X, tau_N)
        """
        tau = np.array([tau_X, tau_N])  # tau = B * u_alloc
        u_alloc = np.matmul(self.Binv, tau)  # u_alloc = inv(B) * tau

        # u_alloc = abs(n) * n --> n = sign(u_alloc) * sqrt(u_alloc)
        n1 = np.sign(u_alloc[0]) * math.sqrt(abs(u_alloc[0]))
        n2 = np.sign(u_alloc[1]) * math.sqrt(abs(u_alloc[1]))

        return n1, n2

    def stepInput(self, t):
        """
        u = stepInput(t) generates propeller step inputs.
        """
        n1 = 100  # rad/s
        n2 = 80

        if t > 30 and t < 100:
            n1 = 80
            n2 = 120
        else:
            n1 = 0
            n2 = 0

        u_control = np.array([n1, n2], float)

        return u_control

    def customTestInput(self, t):
        """
        u = customTestInput(t) generates custom test sequence:
        - 0-30s: propellers off
        - 30-60s: both propellers full speed ahead (straight)
        - 60-90s: left propeller half speed, right full speed
        - 90-100s: propellers off
        """
        if t < 30:
            # 0-30s: propellers off
            n1 = 0
            n2 = 0
        elif t < 60:
            # 30-60s: both propellers full speed ahead (straight)
            n1 = 100  # rad/s
            n2 = 100  # rad/s
        elif t < 90:
            # 60-90s: left propeller half speed, right full speed
            n1 = 50   # rad/s (half speed)
            n2 = 100  # rad/s (full speed)
        else:
            # 90-100s: propellers off
            n1 = 0
            n2 = 0

        u_control = np.array([n1, n2], float)
        return u_control

    def repositioning(self, eta, nu, sample_time):
        """
        eta = repositioning(eta,nu,sample_time) computes the generalized 
        position/Euler angles eta[k+1] for 3-DOF horizontal plane motion.
        
        Note: eta format is [y, x, psi] to match 6-DOF convention where
        eta[0] = y, eta[1] = x (matching BaseController initialization).
        """
        # 3-DOF kinematics (matching 6-DOF Rzyx convention)
        # For horizontal plane: Rzyx(0, 0, psi) gives:
        #   p_dot[0] = y_dot = cos(psi)*u - sin(psi)*v
        #   p_dot[1] = x_dot = sin(psi)*u + cos(psi)*v
        # Note: eta[0] = y, eta[1] = x (matching 6-DOF convention where eta[0:3] = [y, x, z])
        psi = eta[2]
        u = nu[0]
        v = nu[1]
        r = nu[2]

        # Match Rzyx(0, 0, psi) @ [u, v, 0] result:
        y_dot = math.cos(psi) * u - math.sin(psi) * v
        x_dot = math.sin(psi) * u + math.cos(psi) * v
        psi_dot = r

        # Forward Euler integration
        # Note: eta[0] = y, eta[1] = x (matching 6-DOF convention)
        eta[0] = eta[0] + sample_time * y_dot
        eta[1] = eta[1] + sample_time * x_dot
        eta[2] = eta[2] + sample_time * psi_dot

        return eta


if __name__ == "__main__":
    # Minimal self-test
    print("Testing Otter3D class...")
    
    # Instantiate with some current
    otter3d = Otter3D(V_current=0.5, beta_current=30)
    
    # Print matrix shapes
    print(f"M3 shape: {otter3d.M.shape}")
    print(f"D3 shape: {otter3d.D.shape}")
    print(f"M3:\n{otter3d.M}")
    print(f"D3:\n{otter3d.D}")
    
    # Run a few steps with zero inputs
    eta = np.array([0.0, 0.0, 0.0], float)  # [x, y, psi]
    nu = np.array([0.0, 0.0, 0.0], float)  # [u, v, r]
    u_actual = np.array([0.0, 0.0], float)
    u_control = np.array([0.0, 0.0], float)
    dt = 0.1
    
    print(f"\nInitial state:")
    print(f"eta = {eta}")
    print(f"nu = {nu}")
    
    for i in range(3):
        nu, u_actual = otter3d.dynamics(eta, nu, u_actual, u_control, dt)
        eta = otter3d.repositioning(eta, nu, dt)
        print(f"\nStep {i+1}:")
        print(f"nu = {nu}")
        print(f"eta = {eta}")
    
    print("\nTest completed.")
