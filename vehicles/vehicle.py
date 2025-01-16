#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np


class Vehicle:
    name = 'vehicle'
    def __init__(
            self,
            V_current=0,
            serial_number=0,
            shift=None,
            color='b',
            starting_point=None
    ):
        self.V_c = V_current
        self.type = ""
        self.linestyle = '-'
        self.serial_number = serial_number
        if shift is None:
            shift = np.array([0, 0], float)
        if starting_point is None:
            self.starting_point = np.array([0, 0], float) + np.array(shift, float)
        else:
            self.starting_point = np.array(starting_point, float) + np.array(shift, float)
        self.color = color
        self.data_storage = None

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        pass

    def controlAllocation(self, tau_X, tau_N):
        pass

    def repositioning(self, eta, nu, sample_time):
        pass

    def set_data_storage(self, data_storage):
        self.data_storage = data_storage
