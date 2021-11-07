import numpy as np
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
from typing import List, Optional
from multipledispatch import dispatch

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
    def angles_from_rel_position(self, args) -> np.ndarray:
        pass

    @abstractmethod
    def set_default_position(self, args) -> None:
        pass


class LegKinematics(LimbKinematics):
    __slots__ = {"femur",
                 "tibia",
                 "knee",
                 "foot",
                 "_floating"}

    def __init__(self, femur_length, tibia_length):

        super().__init__()

        self.femur = femur_length
        self.tibia = tibia_length
        self._floating = True  # No default position set, so no reference has been set.
        self.origin = None
        self.knee = None
        self.foot = None
        self.vertices = None

    def set_default_position(self, joints_positions: np.ndarray):
        self.origin = joints_positions[0]
        self.knee = joints_positions[1]
        self.foot = joints_positions[2]
        self.vertices = np.array((self.origin, self.knee, self.foot))
        self._floating = False

    @dispatch(np.ndarray, bool, bool)
    def angles_from_rel_position(self, offsets: np.ndarray, foot_fixed, dynamic) -> np.ndarray:
        # todo: Add possibility to specify both origin and foot offset.
        """ Calculate the angles based on the offset of the leg from the current position. The offset can
        can refer to the origin of the leg (shoulder) or the end of the leg (foot)."""

        if not self._floating:
            if dynamic:
                # Dynamic offsets. <- this will eventually be the final version.

                if foot_fixed:
                    self.origin += offsets
                else:
                    self.foot += offsets

                target = self.foot - self.origin

            else:
                # Static offsets.
                if foot_fixed:
                    target = self.foot - offsets - self.origin

                else:
                    target = self.foot + offsets - self.origin

            if np.linalg.norm(target) > self.femur + self.tibia:

                print("Target position unreachable.")
                return np.array((None, None, None))  # This is just for error handling.

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

                result = np.array((leg_ang, femur_ang, -tibia_ang))

                return result
        else:
            raise RuntimeError("Before setting the leg offset, you have to establish a"
                               " reference by calling set_default_position method.")

    @dispatch(np.ndarray, np.ndarray)
    def angles_from_rel_position(self, leg_origin_position: np.ndarray, foot_position: np.ndarray) -> np.ndarray:
        pass



