import matplotlib.pyplot as plt
import numpy as np
from abc import ABC, abstractmethod


"""This is a simplistic environment used to visualize the calculated robot's positions based on matplotlib."""


class BodyPart(ABC):

    """This base class represents the forward kinematics model of limbs which is used to visualize the calculated
    angles from the inverse kinematic model."""

    __slots__ = {"origin",
                 "vertices",
                 "ax",
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
                 "id_",
                 "parent",
                 "_femur_length",
                 "_tibia_length",
                 "_leg_angle",
                 "_femur_angle",
                 "_tibia_angle"}

    # todo: Put angles into a dict.
    # todo: Put limb lengths into a dict.

    def __init__(self, id_: str, parent: BodyPart, ax_: plt.Axes, attach_point: np.ndarray, femur_len, tibia_len,
                 leg_angle, femur_ang, tibia_ang):

        super().__init__()
        self.id_ = id_
        self.ax = ax_
        self.parent = parent
        self._lines = None
        self._lines_vertices = None
        self.origin = attach_point
        self._femur_length = femur_len
        self._tibia_length = tibia_len
        self._leg_angle = leg_angle / 180 * np.pi
        self._femur_angle = femur_ang / 180 * np.pi
        self._tibia_angle = tibia_ang / 180 * np.pi
        self.vertices = {}
        self.joints = np.array((self.origin, (), ()), dtype=object)
        self.update_joints_position(np.array([self._leg_angle, self._femur_angle, self._tibia_angle]))

    def draw(self):

        x_data = np.array([x[0] for x in self.joints])
        y_data = np.array([y[1] for y in self.joints])
        z_data = np.array([z[2] for z in self.joints])

        if not self._lines:
            self._lines = self.ax.plot(x_data,
                                       y_data,
                                       z_data, "b")

            self._lines_vertices = self.ax.scatter3D(x_data,
                                                     y_data,
                                                     z_data,
                                                     edgecolor="blue",
                                                     facecolor="blue")

        else:
            self._lines[0].set_data_3d(x_data,
                                       y_data,
                                       z_data,)

            self._lines_vertices._offsets3d = (x_data,
                                               y_data,
                                               z_data)

    def update_joints_position(self, angles: np.ndarray):  # todo: This needs cleanup.

        self._leg_angle = angles[0]
        self._femur_angle = angles[1]
        self._tibia_angle = angles[2]
        self.origin = self.parent.vertices[self.id_]
        self.joints[0] = self.origin

        joint1_xy = self._femur_length * np.cos(self._femur_angle)
        joint_1 = self.origin + np.array([joint1_xy * np.cos(self._leg_angle),
                                          joint1_xy * np.sin(self._leg_angle),
                                          self._femur_length * np.sin(self._femur_angle)])

        self.joints[1] = joint_1
        joint2_xy = self._tibia_length * np.cos(self._femur_angle + self._tibia_angle)
        joint_2 = joint_1 + np.array([joint2_xy * np.cos(self._leg_angle),
                                      joint2_xy * np.sin(self._leg_angle),
                                      self._tibia_length * np.sin(self._femur_angle + self._tibia_angle)])

        self.joints[2] = joint_2


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
                 "front",
                 "default"}

    def __init__(self, ax_: plt.Axes, length: float, width: float, front: float):
        super().__init__()
        self.length = length
        self.width = width
        self.front = front
        self.origin = np.array([0, 0, 0])
        self.vertices = {"0": self.origin}
        self._create_vertices()
        self._lines = None
        self._lines_vertices = None
        self.ax = ax_
        self.default = self.vertices.copy()

    def _create_vertices(self):

        origin = self.vertices["0"]

        self.vertices["1"] = np.array([origin[0] + self.width/2, origin[1], origin[2]])
        self.vertices["2"] = np.array([origin[0] + self.front/2, origin[1] + self.length/2, origin[2]])
        self.vertices["3"] = np.array([origin[0] - self.front / 2, origin[1] + self.length / 2, origin[2]])
        self.vertices["4"] = np.array([origin[0] - self.width / 2, origin[1], origin[2]])
        self.vertices["5"] = np.array([origin[0] - self.front / 2, origin[1] - self.length / 2, origin[2]])
        self.vertices["6"] = np.array([origin[0] + self.front / 2, origin[1] - self.length / 2, origin[2]])

    def draw(self):

        labels = ["1", "2", "3", "4", "5", "6", "1"]

        x_data = [self.vertices[x][0] for x in labels]
        y_data = [self.vertices[y][1] for y in labels]
        z_data = [self.vertices[z][2] for z in labels]

        if not self._lines:

            self._lines = self.ax.plot(x_data,
                                       y_data,
                                       z_data, "r")

            self._lines_vertices = self.ax.scatter3D(x_data,
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

    def offset_body(self, offset: np.ndarray, dynamic: bool):

        if dynamic:
            for key in self.vertices:
                self.vertices[key] = self.vertices[key] + offset

        else:
            for key in self.vertices:
                self.vertices[key] = self.default[key] + offset


class Hexapod:
    __slots__ = {"_ax",
                 "bodyparts",
                 "_default_elevation",
                 "_leg_origins"}

    def __init__(self, core: Core):
        self._ax = core.ax
        self.bodyparts = {"legs": {}, "core": {"1": core}}  # Leaving room for expansion to other limbs.

    def add_leg(self, femur_len, tibia_len):

        if len(self.bodyparts["legs"]) >= 6:
            raise RuntimeError("Hexapod can have a maximum of 6 legs.")

        labels = ["1", "2", "3", "4", "5", "6"]

        for leg_number in labels:
            if leg_number not in self.bodyparts["legs"]:
                self.bodyparts["legs"][leg_number] = Leg(id_=leg_number,
                                                         parent=self.bodyparts["core"]["1"],
                                                         ax_=self._ax,
                                                         attach_point=self.bodyparts["core"]["1"].vertices[leg_number],
                                                         femur_len=femur_len,
                                                         tibia_len=tibia_len,
                                                         leg_angle=60 * (int(leg_number)-1),
                                                         femur_ang=45,
                                                         tibia_ang=-90)
                # todo: Add possibility to set a default position of the legs.
                break

    def draw(self):
        for part_type in self.bodyparts:
            for part in self.bodyparts[part_type]:
                self.bodyparts[part_type][part].draw()

    def update_leg_positions(self, angles):

        for i, leg in enumerate(self.bodyparts["legs"]):
            self.bodyparts["legs"][leg].update_joints_position(angles[i])
            # print(leg, angles[i])

    def translate_core(self, offset: np.ndarray, dynamic=False):
        self.bodyparts["core"]["1"].offset_body(offset, dynamic)


if __name__ == '__main__':

    fig = plt.figure(figsize=(8, 12))
    ax = fig.add_subplot(111, projection="3d")

    cor = Core(ax, 20, 15, 10)
    robot = Hexapod(cor)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)
    robot.add_leg(15, 20)

    robot.draw()

    plt.show()
