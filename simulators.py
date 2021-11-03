import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from robot import Hexapod, Leg
import numpy as np
import inverse_kinematics as ik
import queue
import time


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


class InverseKinematicsFixedBody:

    def __init__(self, ax: plt.Axes, robot: Hexapod):

        self.bot = robot

        ax.set_title("Inverse kinematics preview, fixed body.")

        lim = 40
        ax.set_xlim3d([-lim, lim])
        ax.set_ylim3d([-lim, lim])
        ax.set_zlim3d([-lim, lim])

        x_offset = plt.axes([0.25, 0.1, 0.65, 0.03])
        y_offset = plt.axes([0.25, 0.15, 0.65, 0.03])
        z_offset = plt.axes([0.25, 0.2, 0.65, 0.03])

        self.x_slider = Slider(x_offset,
                               "X offset",
                               valmin=-10,
                               valmax=10,
                               valinit=0,
                               orientation="horizontal")

        self.y_slider = Slider(y_offset,
                               "Y offset",
                               valmin=-25,
                               valmax=25,
                               valinit=0,
                               orientation="horizontal")

        self.z_slider = Slider(z_offset,
                               "Z offset",
                               valmin=-10,
                               valmax=10,
                               valinit=0,
                               orientation="horizontal")

        self.bot.draw()

        keys = self.bot.bodyparts["legs"].keys()
        self.legs = [ik.LegKinematics(self.bot.bodyparts["legs"][key]) for key in keys]

        self.x_slider.on_changed(self.update)
        self.y_slider.on_changed(self.update)
        self.z_slider.on_changed(self.update)

        plt.show()

    def update(self, _):

        x_offset = self.x_slider.val
        y_offset = self.y_slider.val
        z_offset = self.z_slider.val

        angles = [model.angles_from_rel_position([x_offset, y_offset, z_offset]) for model in self.legs]

        if None not in [ang for result in angles for ang in result]:

            self.bot.update_leg_positions(angles)
            self.bot.draw()

        else:
            pass


class InverseKinematicsFixedLegs:

    def __init__(self, ax: plt.Axes, robot: Hexapod):

        self.bot = robot

        ax.set_title("Inverse kinematics preview, fixed feet.")

        lim = 40
        ax.set_xlim3d([-lim, lim])
        ax.set_ylim3d([-lim, lim])
        ax.set_zlim3d([-lim, lim])

        x_offset = plt.axes([0.25, 0.1, 0.65, 0.03])
        y_offset = plt.axes([0.25, 0.15, 0.65, 0.03])
        z_offset = plt.axes([0.25, 0.2, 0.65, 0.03])

        self.x_slider = Slider(x_offset,
                               "X offset",
                               valmin=-10,
                               valmax=10,
                               valinit=0,
                               orientation="horizontal")

        self.y_slider = Slider(y_offset,
                               "Y offset",
                               valmin=-25,
                               valmax=25,
                               valinit=0,
                               orientation="horizontal")

        self.z_slider = Slider(z_offset,
                               "Z offset",
                               valmin=-10,
                               valmax=10,
                               valinit=0,
                               orientation="horizontal")

        self.bot.draw()

        keys = self.bot.bodyparts["legs"].keys()
        self.legs = [ik.LegKinematics(self.bot.bodyparts["legs"][key]) for key in keys]

        self.x_slider.on_changed(self.update)
        self.y_slider.on_changed(self.update)
        self.z_slider.on_changed(self.update)

        plt.show()

    def update(self, _):

        x_offset = self.x_slider.val
        y_offset = self.y_slider.val
        z_offset = self.z_slider.val

        offsets = [x_offset, y_offset, z_offset]

        angles = [model.angles_from_rel_position(offsets,
                                                 foot_fixed=True) for model in self.legs]

        if None not in [ang for result in angles for ang in result]:

            self.bot.translate_core(offsets)
            self.bot.update_leg_positions(angles)
            self.bot.draw()

        else:
            pass


class Animator:

    def __init__(self, ax: plt.Axes, robot: Hexapod):

        self.bot = robot

        ax.set_title("Animated preview.")

        lim = 40
        ax.set_xlim3d([-lim, lim])
        ax.set_ylim3d([-lim, lim])
        ax.set_zlim3d([-lim, lim])

        self.bot.draw()

        keys = self.bot.bodyparts["legs"].keys()
        self.legs = [ik.LegKinematics(self.bot.bodyparts["legs"][key]) for key in keys]

        plt.show()

        off = 0
        radius = 7

        while True:

            offsets = [radius*np.cos(off), 0 , radius*np.sin(off)]

            off += np.pi/20

            angles = [model.angles_from_rel_position(offsets,
                                                     foot_fixed=True) for model in self.legs]

            if None not in [ang for result in angles for ang in result]:

                self.bot.translate_core(offsets)
                self.bot.update_leg_positions(angles)
                self.bot.draw()

            else:
                pass

            plt.pause(1/60)



