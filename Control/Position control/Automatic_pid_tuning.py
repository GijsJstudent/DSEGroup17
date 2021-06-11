# -*- coding: utf-8 -*-
"""
minimization
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PID import PID ,Data, Controller
from drone_dynamics_vectorized import simulate_drone
import matplotlib.animation as animation
import pickle
from scipy.optimize import minimize


p_rp = 0.8 #proportional gain for roll and pitch controllers
d_rp = 0.1 #proportional gain for roll and pitch controllers
it_rp = 0 #proportional gain for roll and pitch controllers
    
p_xy = 0.1 #proportional gain for x and y controllers
d_xy = 0.2 #proportional gain for x and y controllers
it_xy = 0 #proportional gain for x and y controllers
        
    
test_pid_gains = [p_rp,it_rp,d_rp,p_rp,it_rp,d_rp,0.08,0,0.018,p_xy,it_xy,d_xy,p_xy,it_xy,d_xy,0.8,0.1,0.8] 
#test_pid_gains2 = [p_rp,it_rp,d_rp,p_rp,it_rp,d_rp,0.1,0,0.018,p_xy,it_xy,d_xy,p_xy,it_xy,d_xy,0.8,0.1,0.8]

def func(pid_gains):
    print("1")  
    time_array = np.linspace(0,20,2000)     
    dt = time_array[1] - time_array[0]
    Z_profile = np.linspace(0,20, len(time_array))
    #R = (-1/25)* Z_profile**2 + 4 * Z_profile
    R = np.linspace(0,50, len(time_array))
    Theta = np.cos(time_array/2)
    X_profile = R*np.cos(Theta)
    Y_profile = R*np.sin(Theta)
    
    Yaw_profile = np.ones(len(time_array)) * 0
        
    #Yaw_profile = np.sin(time_array/5) * 1.5
    Yaw_profile = np.ones(len(time_array)) * 0
    
    
    
    Command_matrix = np.vstack((X_profile,Y_profile,Z_profile,Yaw_profile))
    Command_matrix = np.transpose(Command_matrix)
    # print(Command_matrix[0])    
       
    flight_data = simulate_drone(pid_gains, time_array ,Command_matrix)
    e = 0
    for state , reference in zip(flight_data.states, flight_data.references):
        error_xyz = state[0:3] - reference[0:3]
        error_yaw = state[8] - reference[3]
        error = np.hstack((error_xyz,error_yaw))
        e += np.linalg.norm(error)*dt
    return e
obj = minimize(func,test_pid_gains,method='Nelder-Mead',options={'maxiter':100,'return_all': True, 'disp': True})
    
print("optimixed gains: ",obj.x)
print("success: ",        obj.success)
print("message : ",       obj.message)



