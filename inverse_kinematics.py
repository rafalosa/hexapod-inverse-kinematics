import numpy as np
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
from typing import List, Optional
from robot import Leg


#
# class Node:
#
#     def __init__(self, parent: Node, child: Node = None):
#         self.parent = parent
#         self.child = child
#


class LimbKinematics(ABC):
    __slots__ = {"origin",
                 "current_position",
                 "vertices",
                 "end_fixed"}

    def __init__(self):
        pass

    @abstractmethod
    def angles_from_rel_position(self, offsets: List[float]) -> List[float]:
        pass

    @abstractmethod
    def set_default_position(self, position: List[float]) -> None:
        pass


class LegKinematics(LimbKinematics):
    __slots__ = {"femur",
                 "tibia",
                 "knee",
                 "foot"}

    def __init__(self, leg_sim: Leg):
        super().__init__()

        self.origin = leg_sim.joints[0]
        self.femur = leg_sim._femur_length
        self.tibia = leg_sim._tibia_length
        self.knee = leg_sim.joints[1]
        self.foot = leg_sim.joints[2]
        self.vertices = [self.origin, self.knee, self.foot]
        self.current_position = []
        self.end_fixed = False

    def angles_from_rel_position(self, offsets: List[float]) -> List[Optional[float]]:

        # if self.end_fixed:
        #     self.origin = [self.origin[i] + offsets[i] for i in range(3)]
        #
        # else:
        #     self.foot = [self.foot[i] + offsets[i] for i in range(3)]

        target = list(np.array(self.foot) + np.array(offsets) - np.array(self.origin))

        if np.linalg.norm(np.array(target)) > self.femur + self.tibia:

            print("Target position unreachable.")
            return [None, None, None]

        else:

            leg_proj = np.sqrt(target[0]**2 + target[1]**2)
            origin_to_foot = np.sqrt(target[2]**2 + leg_proj**2)

            beta_ang = np.arccos((origin_to_foot**2 + self.femur**2 - self.tibia**2) /
                                 (2 * origin_to_foot * self.femur))

            gamma_ang = np.arccos((origin_to_foot**2 + self.tibia**2 - self.femur**2)/
                                  (2 * self.tibia * origin_to_foot))

            alpha_ang = np.arctan(target[2]/leg_proj)

            leg_ang = np.arccos(target[0] / leg_proj)

            if target[1] < 0:
                leg_ang *= -1

            femur_ang = beta_ang - alpha_ang

            tibia_ang = beta_ang + gamma_ang

            result = [leg_ang, femur_ang, -tibia_ang]

            return result

    def set_default_position(self, position: List[float]):
        pass
