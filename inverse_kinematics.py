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
        print(self.foot)

    def angles_from_rel_position(self, offsets: List[float], foot_fixed=False, dynamic=False) -> List[Optional[float]]:

        """ Calculate the angles based on the offset of the leg from the current position. The offset can
        can refer to the origin of the leg (shoulder) or the end of the leg (foot)."""

        if dynamic:
            # Dynamic offsets. <- this will eventually be the final version.

            if foot_fixed:
                self.origin += np.array(offsets)
            else:
                self.foot += np.array(offsets)

            target = list(np.array(np.array(self.foot) - np.array(self.origin)))

        else:
            # Static offsets.
            if foot_fixed:
                target = list(np.array(self.foot) - np.array(offsets) - np.array(self.origin))

            else:
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

            alpha_ang = np.arcsin(np.abs(target[2])/origin_to_foot)

            leg_ang = np.arccos(target[0] / leg_proj)

            if target[2] > 0:
                alpha_ang *= -1

            femur_ang = beta_ang - alpha_ang

            tibia_ang = beta_ang + gamma_ang

            if target[1] < 0:
                leg_ang *= -1

            result = [leg_ang, femur_ang, -tibia_ang]

            return result

    def set_default_position(self, position: List[float]):
        pass
