# -*- coding: utf-8 -*-
"""

"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PID import PID ,Data, Controller
import matplotlib.animation as animation
import pickle

flight_data = Data()
flight_data.load("flight_data")

X_profile = flight_data.provide("X_profile")

Y_profile = flight_data.provide("Y_profile")

Z_profile = flight_data.provide("Z_profile")

time_array = flight_data.time


# --------------- 3D plotting Animation -------------------
sfactor = 10 # Speed up factor


def update_graph(frame, sfactor, lines):
    i = (frame-1)*sfactor
    lines[0].set_data(tra_x[0:frame * sfactor:sfactor], tra_y[0:frame * sfactor:sfactor])
    lines[0].set_3d_properties(tra_z[0:frame * sfactor:sfactor])
    lines[1].set_data(X_profile[0:frame*sfactor:sfactor], Y_profile[0:frame*sfactor:sfactor])
    lines[1].set_3d_properties(Z_profile[0:frame*sfactor:sfactor])
    lines = lines[0:2]
    # del lines[2]
    # lines[2].set_data([[tra_x[i]-0.5, tra_x[i]+0.5],[tra_y[i]-0.5, tra_y[i]+0.5]])
    # lines[2].set_3d_properties([tra_z[i], tra_z[i]])
    pos = calc_motor_pos(i)
    lines.append(ax.plot([tra_x[i]+pos[0,0], tra_x[i]+pos[2,0]],[tra_y[i]+pos[0,1], tra_y[i]+pos[2,1]],[tra_z[i]+pos[0,2], tra_z[i]+pos[2,2]], 'c')[0])
    lines.append(ax.plot([tra_x[i]+pos[1,0], tra_x[i]+pos[3,0]],[tra_y[i]+pos[1,1], tra_y[i]+pos[3,1]],[tra_z[i]+pos[1,2], tra_z[i]+pos[3,2]], 'c')[0])
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
arm1 = np.array([0.5, 0.5, 0]).T
arm2 = np.array([-0.5, 0.5, 0]).T
arm3 = np.array([-0.5, -0.5, 0]).T
arm4 = np.array([0.5, -0.5, 0]).T

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

    return np.array([pos1,pos2,pos3,pos4])

# Generate lines
print("wqw",tra_x[0])
line1 = ax.plot(tra_x[0:1], tra_y[0:1], tra_z[0:1], 'r')[0]
line2 = ax.plot(X_profile[0:1], Y_profile[0:1], Z_profile[0:1], 'b')[0]
# line3 = ax.plot([tra_x[0]-0.5, tra_x[0]+0.5],[tra_y[0]-0.5, tra_y[0]+0.5],[tra_z[0], tra_z[0]], 'c')[0]
# line4 = ax.plot([tra_x[0]+0.5, tra_x[0]-0.5],[tra_y[0]-0.5, tra_y[0]+0.5],[tra_z[0], tra_z[0]], 'c')[0]
lines = [line1, line2]

line_ani = animation.FuncAnimation(fig, update_graph, int(len(time_array)/sfactor), fargs=(sfactor, lines),
                                   interval=10, blit=True)

plt.show()