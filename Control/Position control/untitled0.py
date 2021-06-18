# -*- coding: utf-8 -*-
"""
Created on Wed Jun 16 09:59:26 2021

Flight envelope

"""

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np



m_empty = 1.51  # kg
T_W = 3

m_pay_max = (2.11*3)-1.51
p_empty = 3.1   # W
eta_bat = 0.95 
DoD = 0.8 
SOL = 0.95
k_ctrl = 0.05
E_bat = 103.6 * 3600
g = 9.81

def Thrust_efficiency(T):
    a = (T*1000/g)/(1652*4)
    efficiency = -0.6664101141 * np.log(a) + 0.9711236673
    return (efficiency * 6.02* 9.81)/(1000)

def flight_time(m_pay,P_pay):
    
    W = (m_empty+m_pay) * g
    T = W
    T_P = Thrust_efficiency(T)
    P_el = T/T_P
    P_tot_el = p_empty + P_el + P_pay
    E_av = E_bat * (eta_bat * DoD * SOL)/(1+k_ctrl)
    t = E_av/P_tot_el
    return t/60
    
print(flight_time(0.3,6))

m_payload = np.linspace(0,m_pay_max,100)
t_flight = flight_time(m_payload,10)

"""
plt.figure()
for payload_power in [0,10,20,30,50,100]:
    
    plt.plot(m_payload,flight_time(m_payload,payload_power),label = str(payload_power)+ " [W]")
    
plt.xlabel('Payload mass [kg]')
plt.ylabel('Flight time [s]')

plt.legend(title='Payload power consumption')
plt.show()

"""
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

# Make data.
m_payload = np.linspace(0, m_pay_max, 100)
P_payload = np.linspace(0, 50, 100)
m_payload2, P_payload2 = np.meshgrid(m_payload, P_payload)


T = flight_time(m_payload2,P_payload2)
print(T)
# Plot the surface.
surf = ax.plot_surface(m_payload2, P_payload2, T, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Customize the z axis.
ax.set_zlim(0, 30)
ax.set_xlabel('Payload mass [kg]')
ax.set_ylabel('Payload power consumption [W]')
ax.set_zlabel('Flight time [min]')
ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
#ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
