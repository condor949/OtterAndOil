from abc import ABC
from collections.abc import Sequence


class BaseController(ABC):
    def __init__(self):
        pass

    def generate_control(self, vehicles, positions) -> Sequence:
        pass
