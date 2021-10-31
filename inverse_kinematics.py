import numpy as np
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
from typing import List


class LimbKinematics(ABC):

    __slots__ = {"_origin",
                 "current_position"}

    def __init__(self):
        pass

    @abstractmethod
    def angles_from_position(self, position: List[float]) -> List[float]:
        pass


class LegKinematics(LimbKinematics):

    __slots__ = {"femur",
                 "tibia"}

    def __init__(self, attach_point, femur_length, tibia_length):
        super().__init__()
        self._origin = attach_point
        self.femur = femur_length
        self.tibia = tibia_length
        self.current_position = []

    def angles_from_position(self, position: List[float]) -> List[float]:
        pass
