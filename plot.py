from numpy import pi
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from arm import armPoints, armPointsCoolerVersion

# Create the figure and the line that we will manipulate
fig, ax = plt.subplot_mosaic([['up'], ['down']])


def update_ax(arm_points):
    for graph in ax.values():
        graph.cla()
        graph.set_xlim([-3, 3])
        graph.set_aspect('equal')

    ax['up'].set_ylim([-3, 3])
    ax['up'].set_title("side")

    ax['down'].set_ylim([-3, 3])
    ax['down'].set_title("top")

    arm_x = list(pt[0] for pt in arm_points)
    arm_y = list(pt[1] for pt in arm_points)
    arm_z = list(pt[2] for pt in arm_points)

    ax['up'].plot(arm_x[:-1], arm_y[:-1], 'gray')
    ax['up'].plot([arm_x[2], arm_x[4]], [arm_y[2], arm_y[4]], 'gray')
    ax['up'].scatter(arm_x, arm_y, 2, 'gray')

    ax['down'].plot(arm_x[:-1], arm_z[:-1], 'gray')
    ax['down'].plot([arm_x[2], arm_x[4]], [arm_z[2], arm_z[4]], 'gray')
    ax['down'].scatter(arm_x, arm_z, 2, 'gray')


update_ax([np.array([0, 0.8, 0]), np.array([0, 1.85, 0]),
           np.array([0, 2.45, 0]),
           np.array([0, 3.45, 0]), np.array([0, 3.45, 0])])

# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.6, bottom=0.15, hspace=0.4)

# Make a horizontal slider to control the shoulder joint
ax_base = fig.add_axes([0.45, 0.25, 0.0225, 0.63])
base_angle = Slider(
    ax=ax_base,
    label='Base',
    valmin=0,
    valmax=180,
    valinit=0,
    orientation="vertical"
)

# Make a horizontal slider to control the shoulder joint
ax_shoulder = fig.add_axes([0.35, 0.25, 0.0225, 0.63])
shoulder_angle = Slider(
    ax=ax_shoulder,
    label='Shoulder',
    valmin=-90,
    valmax=90,
    valinit=0,
    orientation="vertical"
)

# Make a vertically oriented slider to control the elbow joint
ax_elbow = fig.add_axes([0.25, 0.25, 0.0225, 0.63])
elbow_angle = Slider(
    ax=ax_elbow,
    label="Elbow",
    valmin=-90,
    valmax=90,
    valinit=0,
    orientation="vertical"
)

# Make a vertically oriented slider to control the wrist joint
ax_wrist = fig.add_axes([0.15, 0.25, 0.0225, 0.63])
wrist_angle = Slider(
    ax=ax_wrist,
    label="Wrist",
    valmin=-90,
    valmax=90,
    valinit=0,
    orientation="vertical"
)

# Make a vertically oriented slider to control the wrist joint
ax_gripper = fig.add_axes([0.05, 0.25, 0.0225, 0.63])
gripper_angle = Slider(
    ax=ax_gripper,
    label="Gripper",
    valmin=0,
    valmax=30,
    valinit=0,
    orientation="vertical"
)


# The function to be called anytime a slider's value changes
def update(val):
    arm_points = armPointsCoolerVersion([shoulder_angle.val/180*pi,
                                         elbow_angle.val/180*pi,
                                         wrist_angle.val/180*pi+pi/2,
                                         gripper_angle.val/180*pi,
                                         base_angle.val/180*pi+pi/2])
    update_ax(arm_points)
    fig.canvas.draw_idle()


# register the update function with each slider
shoulder_angle.on_changed(update)
elbow_angle.on_changed(update)
wrist_angle.on_changed(update)
base_angle.on_changed(update)
gripper_angle.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')


def reset(event):
    shoulder_angle.reset()
    elbow_angle.reset()
    wrist_angle.reset()
    base_angle.reset()
    gripper_angle.reset()


button.on_clicked(reset)

plt.show()
