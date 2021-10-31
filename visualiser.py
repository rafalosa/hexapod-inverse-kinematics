import matplotlib.pyplot as plt
import numpy as np
from typing import List
from abc import ABC, abstractmethod
from matplotlib.widgets import Slider


"""This is a simplistic environment used to visualize the calculated robot's positions based on matplotlib."""


# todo: Convert lists into np.ndarray.

class BodyPart(ABC):

    """This base class represents the forward kinematics model of limbs which is used to visualize the calculated
    angles from the inverse kinematic model."""

    __slots__ = {"_origin",
                 "vertices",
                 "_ax",
                 "_lines",
                 "_lines_vertices"}

    def __init__(self):
        self.vertices = {}

    @abstractmethod
    def draw(self, **kwargs):
        pass

    @abstractmethod
    def update_joints_position(self, **kwargs):
        pass


class Leg(BodyPart):
    __slots__ = {"joints",
                 "_femur_length",
                 "_tibia_length",
                 "_leg_angle",
                 "_femur_angle",
                 "_tibia_angle"}

    def __init__(self, ax_: plt.Axes, attach_point: List[float], femur_len, tibia_len, leg_angle, femur_ang, tibia_ang):
        super().__init__()
        self._ax = ax_
        self._lines = None
        self._lines_vertices = None

        self._origin = attach_point
        self._femur_length = femur_len
        self._tibia_length = tibia_len
        self._leg_angle = leg_angle / 180 * np.pi
        self._femur_angle = femur_ang / 180 * np.pi
        self._tibia_angle = tibia_ang / 180 * np.pi
        self.vertices = {}
        self.joints = [self._origin, [], []]
        self.update_joints_position([self._leg_angle, self._femur_angle, self._tibia_angle])

    def draw(self, ax_: plt.Axes):

        point1 = self.joints[0]
        point2 = self.joints[1]
        point3 = self.joints[2]

        if not self._lines:
            self._lines = ax_.plot([point1[0], point2[0], point3[0]],
                                   [point1[1], point2[1], point3[1]],
                                   [point1[2], point2[2], point3[2]], "b")

            self._lines_vertices = ax_.scatter3D([point1[0], point2[0], point3[0]],
                                                 [point1[1], point2[1], point3[1]],
                                                 [point1[2], point2[2], point3[2]],
                                                 edgecolor="blue",
                                                 facecolor="blue")

        else:
            self._lines[0].set_data_3d([point1[0], point2[0], point3[0]],
                                       [point1[1], point2[1], point3[1]],
                                       [point1[2], point2[2], point3[2]])

            self._lines_vertices._offsets3d = ([point1[0], point2[0], point3[0]],
                                               [point1[1], point2[1], point3[1]],
                                               [point1[2], point2[2], point3[2]])

    def update_joints_position(self, angles: List[float]):

        self._leg_angle = angles[0]
        self._femur_angle = angles[1]
        self._tibia_angle = angles[2]

        joint1_xy = self._femur_length * np.cos(self._femur_angle)
        joint_1 = np.array(self._origin) + np.array([joint1_xy * np.cos(self._leg_angle),
                                                     joint1_xy * np.sin(self._leg_angle),
                                                     self._femur_length * np.sin(self._femur_angle)])

        self.joints[1] = list(joint_1)
        joint2_xy = self._tibia_length * np.cos(self._tibia_angle)
        joint_2 = joint_1 + np.array([joint2_xy * np.cos(self._leg_angle),
                                      joint2_xy * np.sin(self._leg_angle),
                                      self._tibia_length * np.sin(self._tibia_angle)])

        self.joints[2] = list(joint_2)


class Core(BodyPart):
    """The hexapod core is a hexagon with a desired width and length. The coordinates of vertices are calculated
    from the provided parameters."""

    #    _____________  l
    #   /3    2\     |  e
    #  /        \    |  n
    # |4   .0   1|   |  g
    # |\        /|   |  t
    # |_\5____6/_|____  h
    #   width

    __slots__ = {"length",
                 "width",
                 "front"}

    def __init__(self, ax_: plt.Axes, length: float, width: float, front: float):
        super().__init__()
        self.length = length
        self.width = width
        self.front = front
        self._origin = [0, 0, 0]
        self.vertices = {"0": self._origin}
        self._create_vertices()
        self._lines = None
        self._lines_vertices = None
        self._ax = ax

    def _create_vertices(self):

        origin = self.vertices["0"]

        self.vertices["1"] = [origin[0] + self.width/2, origin[1], origin[2]]
        self.vertices["2"] = [origin[0] + self.front/2, origin[1] + self.length/2, origin[2]]
        self.vertices["3"] = [origin[0] - self.front / 2, origin[1] + self.length / 2, origin[2]]
        self.vertices["4"] = [origin[0] - self.width / 2, origin[1], origin[2]]
        self.vertices["5"] = [origin[0] - self.front / 2, origin[1] - self.length / 2, origin[2]]
        self.vertices["6"] = [origin[0] + self.front / 2, origin[1] - self.length / 2, origin[2]]

    def draw(self, ax_):

        labels = ["1", "2", "3", "4", "5", "6", "1"]

        x_data = [self.vertices[x][0] for x in labels]
        y_data = [self.vertices[y][1] for y in labels]
        z_data = [self.vertices[z][2] for z in labels]

        if not self._lines:

            self._lines = ax_.plot(x_data,
                                   y_data,
                                   z_data, "r")

            self._lines_vertices = ax_.scatter3D(x_data,
                                                 y_data,
                                                 z_data,
                                                 edgecolor="red",
                                                 facecolor="red")
        else:
            self._lines[0].set_data_3d(x_data,
                                       y_data,
                                       z_data)

            self._lines_vertices._offsets3d = (x_data,
                                               y_data,
                                               z_data)

    def update_joints_position(self, position):
        pass


class Hexapod:
    __slots__ = {"_ax",
                 "_bodyparts",
                 "_default_elevation",
                 "_leg_origins"}

    def __init__(self, ax_: plt.Axes, core: Core):
        self._ax = ax_
        self._bodyparts = {"legs": {}, "core": {"1": core}}  # Leaving room for expansion to other limbs.

    def add_leg(self, femur_len, tibia_len):

        if len(self._bodyparts["legs"]) >= 6:
            raise RuntimeError("Hexapod can have a maximum of 6 legs.")

        labels = ["1", "2", "3", "4", "5", "6"]

        for leg_number in labels:
            if leg_number not in self._bodyparts["legs"]:
                self._bodyparts["legs"][leg_number] = Leg(self._ax,
                                                          self._bodyparts["core"]["1"].vertices[leg_number],
                                                          femur_len,
                                                          tibia_len,
                                                          60*(int(leg_number)-1),
                                                          0,
                                                          0)
                break

    def draw(self, ax_: plt.Axes):
        for part_type in self._bodyparts:
            for part in self._bodyparts[part_type]:
                self._bodyparts[part_type][part].draw(ax_)

    def update_leg_positions(self, positions):

        for i, leg in enumerate(self._bodyparts["legs"]):
            self._bodyparts["legs"][leg].update_joints_position(positions[i])


class ForwardKinematicsPreview:

    def __init__(self, ax_:plt.Axes, robot: Hexapod):
        self.robot = robot
        self.ax = ax_

        ax_leg_ang = plt.axes([0.25, 0.1, 0.65, 0.03])
        ax_fem_ang = plt.axes([0.25, 0.15, 0.65, 0.03])
        ax_tib_ang = plt.axes([0.25, 0.2, 0.65, 0.03])

        self.leg_ang = Slider(ax=ax_leg_ang,
                              label="Leg base angle",
                              valmin=-90,
                              valmax=90,
                              valinit=0,
                              orientation="horizontal")

        self.fem_ang = Slider(ax=ax_fem_ang,
                              label="Femur angle",
                              valmin=-120,
                              valmax=120,
                              valinit=0,
                              orientation="horizontal")

        self.tib_ang = Slider(ax=ax_tib_ang,
                              label="Tibia angle",
                              valmin=-90,
                              valmax=90,
                              valinit=0,
                              orientation="horizontal")

        ax.set_xlim3d([-20, 20])
        ax.set_ylim3d([-20, 20])
        ax.set_zlim3d([-20, 20])

        self.leg_ang.on_changed(self.update)
        self.fem_ang.on_changed(self.update)
        self.tib_ang.on_changed(self.update)

        self.robot.draw(self.ax)

        plt.show()

    def update(self, _):

        angles = [[(ang * 60 + self.leg_ang.val) / 180 * np.pi,
                   self.fem_ang.val / 180 * np.pi,
                   self.tib_ang.val / 180 * np.pi] for ang in range(6)]

        robot.update_leg_positions(angles)
        robot.draw(ax)


if __name__ == '__main__':

    fig = plt.figure(figsize=(8, 10))
    ax = fig.add_subplot(111, projection="3d")

    cor = Core(ax, 20, 15, 10)
    robot = Hexapod(ax, cor)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)

    ForwardKinematicsPreview(ax, robot)
