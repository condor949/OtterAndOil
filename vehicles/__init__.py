#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from .vehicle import Vehicle
from .otter import Otter
from .dubins import Dubins

vehicle_instance = {}

# Register decorator
def register_class(cls):
    vehicle_instance[cls.name] = cls
    return cls

def create_instance(class_name: str, **arguments) -> Vehicle:
    if class_name in vehicle_instance:
        return vehicle_instance[class_name](**arguments)
    else:
        raise ValueError(f"Unknown class name: {class_name}")

register_class(Otter)
register_class(Dubins)