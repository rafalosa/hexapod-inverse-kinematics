import matplotlib.pyplot as plt
import simulators
from robot import Hexapod, Core, Leg
from matplotlib.widgets import Slider
import inverse_kinematics as ik


if __name__ == '__main__':

    fig = plt.figure(figsize=(10, 12))
    ax = fig.add_subplot(111, projection="3d")

    femur = 10
    tibia = 12

    body = Core(ax, 20, 10, 5)
    bot = Hexapod(body)
    bot.add_leg(femur, tibia)
    bot.add_leg(femur, tibia)
    bot.add_leg(femur, tibia)
    bot.add_leg(femur, tibia)
    bot.add_leg(femur, tibia)
    bot.add_leg(femur, tibia)

    simulators.InverseKinematicsFixedBody(ax, bot)


