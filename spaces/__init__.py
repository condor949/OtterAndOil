from .BaseSpace import *
from .GaussianSpace import *
from .ParabolicSpace import *

space_instance = {}

# Register decorator
def register_class(cls):
    space_instance[cls.name] = cls
    return cls

def create_instance(class_name: str, **arguments) -> BaseSpace:
    if class_name in space_instance:
        return space_instance[class_name](**arguments)
    else:
        raise ValueError(f"Unknown class name: {class_name}")

register_class(Gaussian3DSpace)
register_class(Parabolic3DSpace)