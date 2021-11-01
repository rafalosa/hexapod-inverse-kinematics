import matplotlib.pyplot as plt
import simulators
from robot import Hexapod, Core, Leg
from matplotlib.widgets import Slider
import inverse_kinematics as ik

def update(_):

    xoff = x_slider.val
    yoff = y_slider.val
    zoff = z_slider.val

    angs = model.angles_from_rel_position([xoff, yoff, zoff])

    limb.update_joints_position(angs)

    limb.draw()


if __name__ == '__main__':

    fig = plt.figure(figsize=(10, 12))
    ax = fig.add_subplot(111, projection="3d")
    femur = 10
    tibia = 12

    # body = Core(ax, 20, 10, 5)
    # bot = Hexapod(body)
    # bot.add_leg(femur, tibia)
    # bot.add_leg(femur, tibia)
    # bot.add_leg(femur, tibia)
    # bot.add_leg(femur, tibia)
    # bot.add_leg(femur, tibia)
    # bot.add_leg(femur, tibia)
    # bot.draw()
    #
    # sim = simulators.ForwardKinematicsPreview(ax, bot)

    limb = Leg(ax, [0, 0, 0], femur, tibia, 0, 0, 0)
    foot = limb.joints[2]
    print(foot)

    lim = 15
    ax.set_xlim3d([-lim, lim])
    ax.set_ylim3d([-lim, lim])
    ax.set_zlim3d([-lim, lim])

    x_offset = plt.axes([0.25, 0.1, 0.65, 0.03])
    y_offset = plt.axes([0.25, 0.15, 0.65, 0.03])
    z_offset = plt.axes([0.25, 0.2, 0.65, 0.03])

    x_slider = Slider(x_offset,
                      "X offset",
                      valmin=-10,
                      valmax=10,
                      valinit=0,
                      orientation="horizontal")

    y_slider = Slider(y_offset,
                      "Y offset",
                      valmin=-25,
                      valmax=25,
                      valinit=0,
                      orientation="horizontal")

    z_slider = Slider(z_offset,
                      "Z offset",
                      valmin=-10,
                      valmax=10,
                      valinit=0,
                      orientation="horizontal")

    limb.draw()
    model = ik.LegKinematics(limb._origin, femur, tibia, 0)

    x_slider.on_changed(update)
    y_slider.on_changed(update)
    z_slider.on_changed(update)

    plt.show()