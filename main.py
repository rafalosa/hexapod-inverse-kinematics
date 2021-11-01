import matplotlib.pyplot as plt
import simulators
from robot import Hexapod, Core


if __name__ == '__main__':

    fig = plt.figure(figsize=(10, 12))
    ax = fig.add_subplot(111, projection="3d")

    body = Core(ax, 20, 30, 20)
    bot = Hexapod(ax, body)
    bot.add_leg(30, 40)
    bot.add_leg(30, 40)
    bot.add_leg(30, 40)
    bot.add_leg(30, 40)
    bot.add_leg(30, 40)
    bot.add_leg(30, 40)

    sim = simulators.ForwardKinematicsPreview(ax, bot)