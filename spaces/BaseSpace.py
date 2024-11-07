from abc import ABC
from collections.abc import Sequence


class BaseSpace(ABC):
    def __init__(self):
        pass

    def get_intensity(self, x_current, y_current):
        pass

    def get_X(self) -> Sequence:
        pass

    def get_Y(self) -> Sequence:
        pass

    def get_Z(self) -> Sequence:
        pass