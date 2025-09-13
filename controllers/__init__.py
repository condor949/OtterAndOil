from .BaseController import *
from .SwarmController import *
from .IntensityBasedController import *
from .IntensityAndLinearVelocityController import *
from .IntensityAndLinearVelocityPIDController import *

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
register_class(IntensityAndLinearVelocityController)
register_class(IntensityAndLinearVelocityPIDController)
