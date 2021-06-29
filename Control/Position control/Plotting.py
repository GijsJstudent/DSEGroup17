import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np




max_time = 50
dt = 0.001
N_time = int(max_time/dt)
time_stamps = 20000
time_array = np.linspace(0,max_time,N_time)     

X_profile = np.linspace(0,20, len(time_array))
Theta = np.linspace(0,20, len(time_array))
Y_profile = 10*np.cos(Theta)
Z_profile = 10*np.sin(Theta) + 10
Yaw_profile = np.linspace(0,20, len(time_array))*0

length =0

for i in range(len(X_profile[:-1])):
    dx = X_profile[i+1] -  X_profile[i]
    dy = Y_profile[i+1] -  Y_profile[i]
    dz = Z_profile[i+1] -  Z_profile[i]
    dl = (dx**2+ dy**2 + dz**2)**0.5
    length += dl
print(length)
x_wind = np.ones(len(time_array)) * 0
y_wind = np.ones(len(time_array)) * 0
z_wind = np.ones(len(time_array)) * 0


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#Axes3D.plot(ax, flight_data.provide(0)[0], flight_data.provide(1)[0], flight_data.provide(2)[0])
Axes3D.plot(ax, X_profile, Y_profile, Z_profile)
plt.show()