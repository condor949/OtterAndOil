from .BaseController import *
from .SwarmController import *
from .IntensityBasedController import *

controller_instance = {}

# Register decorator
def register_class(cls):
    controller_instance[cls.name] = cls
    return cls

def create_instance(class_name: str, **arguments) -> BaseController:
    if class_name in controller_instance:
        return controller_instance[class_name](**arguments)
    else:
        raise ValueError(f"Unknown class name: {class_name}")

register_class(SwarmController)
register_class(IntensityBasedController)