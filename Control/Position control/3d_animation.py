# -*- coding: utf-8 -*-
"""

"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PID import PID ,Data, Controller
import matplotlib.animation as animation
# from matplotlib.animation import FuncAnimation, PillowWriter
# import networkx as nx
import pickle

flight_data = Data()
flight_data.load("flight_data")

X_profile = flight_data.provide("X_profile")[0]

Y_profile = flight_data.provide("Y_profile")[0]

Z_profile = flight_data.provide("Z_profile")[0]

Yaw_profile = flight_data.provide("Yaw_profile")[0]

time_array = flight_data.time


# --------------- 3D plotting Animation -------------------
sfactor = 10 # Speed up factor
show_lines = 1
arm1 = np.array([0.5, 0.5, 0]).T
arm2 = np.array([-0.5, 0.5, 0]).T
arm3 = np.array([-0.5, -0.5, 0]).T
arm4 = np.array([0.5, -0.5, 0]).T
head = np.array([0, 1, 0]).T

def update_graph(frame, sfactor, lines):
    i = (frame-1)*sfactor
    lines[0].set_data(tra_x[0:frame * sfactor:sfactor], tra_y[0:frame * sfactor:sfactor])
    lines[0].set_3d_properties(tra_z[0:frame * sfactor:sfactor])
    lines[1].set_data(X_profile[0:frame*sfactor:sfactor], Y_profile[0:frame*sfactor:sfactor])
    lines[1].set_3d_properties(Z_profile[0:frame*sfactor:sfactor])

    pos = calc_motor_pos(i) # Calculate positions of the arms
    lines[2].set_data(np.array([tra_x[i]+pos[0,0], tra_x[i]+pos[2,0]]),np.array([tra_y[i]+pos[0,1], tra_y[i]+pos[2,1]]))
    lines[2].set_3d_properties(np.array([tra_z[i]+pos[0,2], tra_z[i]+pos[2,2]]))
    lines[3].set_data(np.array([tra_x[i]+pos[1,0], tra_x[i]+pos[3,0]]),np.array([tra_y[i]+pos[1,1], tra_y[i]+pos[3,1]]))
    lines[3].set_3d_properties(np.array([tra_z[i]+pos[1,2], tra_z[i]+pos[3,2]]))
    lines[4].set_data(np.array([tra_x[i], tra_x[i] + pos[4, 0]]),
                      np.array([tra_y[i], tra_y[i] + pos[4, 1]]))
    lines[4].set_3d_properties(np.array([tra_z[i], tra_z[i] + pos[4, 2]]))
    return lines

fig = plt.figure()
ax = Axes3D(fig)

ax.set_xlim3d([-10, 10])
ax.set_xlabel('X')

ax.set_ylim3d([-10, 10])
ax.set_ylabel('Y')

ax.set_zlim3d([0.0, 10])
ax.set_zlabel('Z')

ax.set_title('3D Test')

# Trajectory unpacked
tra_x = flight_data.provide(0)[0]
tra_y = flight_data.provide(1)[0]
tra_z = flight_data.provide(2)[0]
roll = flight_data.provide(6)[0]
pitch = flight_data.provide(7)[0]
yaw = flight_data.provide(8)[0]


def calc_motor_pos(i):
    # define rotation matrices

    # R_1_E, R_2_1, R_b_2 are euler angle rotations around z,y,x axis
    R_1_E = np.array([[np.cos(yaw[i]), np.sin(yaw[i]), 0],
                      [-np.sin(yaw[i]), np.cos(yaw[i]), 0],
                      [0, 0, 1]])

    R_2_1 = np.array([[np.cos(pitch[i]), 0, -np.sin(pitch[i])],
                      [0, 1, 0],
                      [np.sin(pitch[i]), 0, np.cos(pitch[i])]])

    R_b_2 = np.array([[1, 0, 0],
                      [0, np.cos(roll[i]), np.sin(roll[i])],
                      [0, -np.sin(roll[i]), np.cos(roll[i])]])

    # R_b_E = R_b_2 @ (R_2_1 @ R_1_E) #transformation from E frame to b frame

    R_E_1 = np.linalg.inv(R_1_E)
    R_1_2 = np.linalg.inv(R_2_1)
    R_2_b = np.linalg.inv(R_b_2)

    R_E_b = R_E_1 @ (R_1_2 @ R_2_b)  # transformation from b frame to E frame
    pos1 = R_E_b @ arm1
    pos2 = R_E_b @ arm2
    pos3 = R_E_b @ arm3
    pos4 = R_E_b @ arm4
    headline = R_E_b @ head

    return np.array([pos1,pos2,pos3,pos4,headline])

# Generate lines
line1 = ax.plot(tra_x[0:1], tra_y[0:1], tra_z[0:1], 'r')[0]
line2 = ax.plot(X_profile[0:1], Y_profile[0:1], Z_profile[0:1], 'b')[0]
line3 = ax.plot([tra_x[0]-0.5, tra_x[0]+0.5],[tra_y[0]-0.5, tra_y[0]+0.5],[tra_z[0], tra_z[0]], 'c')[0]
line4 = ax.plot([tra_x[0]+0.5, tra_x[0]-0.5],[tra_y[0]-0.5, tra_y[0]+0.5],[tra_z[0], tra_z[0]], 'c')[0]
line5 = ax.plot([tra_x[0], tra_x[0]],[tra_y[0], tra_y[0]+0.5],[tra_z[0], tra_z[0]], 'g')[0]
lines = [line1, line2, line3, line4, line5]


line_ani = animation.FuncAnimation(fig, update_graph, int(len(time_array)/sfactor), fargs=(sfactor, lines),
                                   interval=10, blit=False)
# line_ani.save('test1.gif', writer='imagemagick', fps=30)
plt.show()


# 3D plotting trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

Axes3D.plot(ax, flight_data.provide(0)[0], flight_data.provide(1)[0], flight_data.provide(2)[0])
Axes3D.plot(ax, X_profile, Y_profile, Z_profile)
plt.show()

fig, axs = plt.subplots(4, 3, figsize=(14, 7), sharex=True)
time_array = flight_data.provide(1)[1]
print("time array len", len(time_array))
print("asasas", len(flight_data.provide(1)[0]))
for ax, i in zip(axs.flat, range(12)):
    ax.plot(time_array, flight_data.provide(i)[0])

    if i == 0:
        ax.plot(time_array, X_profile)

    if i == 1:
        ax.plot(time_array, Y_profile)

    if i == 2:
        ax.plot(time_array, Z_profile)

    if i == 8:
        ax.plot(time_array, Yaw_profile)

    ax.set(ylabel=flight_data.provide(i)[2])

plt.show()