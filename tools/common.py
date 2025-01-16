import inspect


def plotting_all(obj, **arguments):
    for name, method in inspect.getmembers(obj, predicate=inspect.ismethod):
        if name.startswith('plotting_'):
            method(**arguments)

def store_all(obj, **arguments):
    for name, method in inspect.getmembers(obj, predicate=inspect.ismethod):
        if name.startswith('store_'):
            method(**arguments)

def get_class_attributes(obj):
    # Use dict to get all instance attributes
    attributes = {k: v for k, v in obj.__dict__.items() if not inspect.ismethod(v)}
    return attributes
