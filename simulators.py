import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from robot import Hexapod
import numpy as np


class ForwardKinematicsPreview:

    def __init__(self, ax_: plt.Axes, robot: Hexapod):
        self.robot = robot
        self.ax = ax_
        self.ax.set_title("Forward kinematics preview")

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
                              valmin=-120,
                              valmax=120,
                              valinit=0,
                              orientation="horizontal")

        lim = 40
        self.ax.set_xlim3d([-lim, lim])
        self.ax.set_ylim3d([-lim, lim])  # todo: Automate adjusting the limits, since set_aspect doesn't work on 3d axes.
        self.ax.set_zlim3d([-lim, lim])

        self.leg_ang.on_changed(self.update)
        self.fem_ang.on_changed(self.update)
        self.tib_ang.on_changed(self.update)

        self.robot.draw()

        plt.show()

    def update(self, _):

        angles = [[(ang * 60 + self.leg_ang.val) / 180 * np.pi,
                   self.fem_ang.val / 180 * np.pi,
                   self.tib_ang.val / 180 * np.pi] for ang in range(6)]

        self.robot.update_leg_positions(angles)
        self.robot.draw()
