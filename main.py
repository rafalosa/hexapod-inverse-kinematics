import matplotlib.pyplot as plt
import simulators
from robot import Hexapod, Core, Leg


if __name__ == '__main__':

    with plt.ion():

        fig = plt.figure(figsize=(10, 12))
        ax = fig.add_subplot(111, projection="3d")

        femur = 20
        tibia = 40

        body = Core(ax, 20, 20, 15)
        bot = Hexapod(body)
        bot.add_leg(femur, tibia)
        bot.add_leg(femur, tibia)
        bot.add_leg(femur, tibia)
        bot.add_leg(femur, tibia)
        bot.add_leg(femur, tibia)
        bot.add_leg(femur, tibia)

        simulators.Animator(ax, bot)

