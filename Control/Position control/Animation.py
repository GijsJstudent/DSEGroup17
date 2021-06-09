# -*- coding: utf-8 -*-
"""
Created on Wed Jun  9 09:41:33 2021

Animation
"""
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from PID import Data
# Fixing random state for reproducibility
np.random.seed(19680801)


def generate_frame(num, states):
    


# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)
data = [Gen_RandLine(25, 3) for index in range(50)]

# Creating fifty line objects.
# NOTE: Can't pass empty arrays into 3d version of plot()
lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]

# Setting the axes properties
ax.set_xlim3d([0.0, 1.0])
ax.set_xlabel('X')

ax.set_ylim3d([0.0, 1.0])
ax.set_ylabel('Y')

ax.set_zlim3d([0.0, 1.0])
ax.set_zlabel('Z')

ax.set_title('3D Test')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, 25, fargs=(data, lines),
                                   interval=50, blit=False)
line_ani.save('myAnimation.gif', writer='imagemagick', fps=30)
plt.show()

