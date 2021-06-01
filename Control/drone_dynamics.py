# -*- coding: utf-8 -*-
"""
Created on Sun May 30 21:06:41 2021

3D drone dynamics v1.0
"""
import numpy as np
import matplotlib.pyplot as plt 
from PID import PID
# defining drone parameters 

# unit vectors in x,y,z directions (might be useful later)
bx= np.array([1,0,0])
by= np.array([0,1,0])
bz= np.array([0,0,1])

I = [[2.5*10**-3, 0,          0       ], # drone moment of inertia 
     [0,          2.5*10**-3, 0       ],
     [0,          0,          5*10**-3]]
# constants identification 
I_prop = 2* 10**-5# propeller moment of inertia 
w_prop_max = 1000 # maximum angular velocity of propellers
M = 0.4 # [kg] mass of the drone 
x_prop = 0.178 #[m] x distance from c.g. to propeller
y_prop = 0.178 #[m] y distance from c.g. to propeller
g = 9.81 # gravitational constant 
c_m = 2.5*(10**-7) # torque coefficient of propeller T = c_m * w^2
c_t = 8*(10**-6) # thrust coefficient of propeller F = c_m * w^2
S = 0.03 # reference area of the drone 
C_d = 0.5 #drag coefficient
rho = 1.225 # density of air 
# function that calculates derivative from the state 

def derivative(state,inputs):
# state and inputs are lists with length 12 and 4, next lines unpack them
    x,y,z,v_x,v_y,v_z = state[0:6]
    roll, pitch, yaw, w_x, w_y, w_z = state[6:12]
    
    #prevent unrealistic control inputs, by limiting the rpm of morors to max_rpm  
    
    w_prop1, w_prop2, w_prop3, w_prop4 = inputs
    #put some values in vector form
    w = np.array([w_x,w_y,w_z])
    v = np.array([v_x,v_y,v_z])
    # define rotation matrices 
    
    # R_1_E, R_2_1, R_b_2 are euler angle rotations around z,y,x axis 
    R_1_E = np.array([[ np.cos(yaw), np.sin(yaw), 0],
                      [-np.sin(yaw), np.cos(yaw), 0],
                      [           0,           0, 1]])
    
    R_2_1 = np.array([[np.cos(pitch), 0, -np.sin(pitch)],
                      [            0, 1,              0],
                      [np.sin(pitch), 0,  np.cos(pitch)]])
    
    
    R_b_2 = np.array([[1,             0,            0],
                      [0,  np.cos(roll), np.sin(roll)],
                      [0, -np.sin(roll), np.cos(roll)]])
    
    R_b_E = R_b_2 @ (R_2_1 @ R_1_E) #transformation from E frame to b frame
    
    R_E_1 = np.linalg.inv(R_1_E)
    R_1_2 = np.linalg.inv(R_2_1)
    R_2_b = np.linalg.inv(R_b_2)
    
    R_E_b = R_E_1 @ (R_1_2 @ R_2_b) # transformation from b frame to E frame
    
    
    W1 = np.linalg.inv(np.array([[1,  0           , -np.sin(pitch)             ],
                                [0,  np.cos(roll),  np.cos(pitch)*np.sin(roll)],
                                [0, -np.sin(roll),  np.cos(pitch)*np.cos(roll)]]))
    
    W2 = np.array([[1,  np.tan(pitch)*np.sin(roll),  np.tan(pitch)*np.cos(roll)],
                   [0,  np.cos(roll)              , -np.sin(roll)              ],
                   [0,  np.sin(roll)/np.cos(pitch),  np.cos(roll)/np.cos(pitch)]])
    
    F_g = np.array([0,0,-M*g]) #force of gravity in E frame
    
    F_thrust = R_E_b @ np.array([0,0,c_t*np.sum(np.square(inputs))]) # thrust force in E frame
    
    F_a = (-1*v) * np.linalg.norm(v) * 0.5 * rho * C_d # aerodynamic force
    #F_a = 0
    v_dot = (1/M) * (F_g + F_a + F_thrust) # derivative of velocity in E frame in vector form
    
    p_dot = v # derivative of position in E frame in vector form

    #Gyroscopic torque caused by the spinning props in b frame in vector form
    # !!!!!Check derivations again, high chance oh mistake !!!!!
    G = I_prop * np.array([w_y*(-w_prop1+w_prop2-w_prop3+w_prop4),w_x*(w_prop1-w_prop2+w_prop3-w_prop4),0])
    
    # torque produced by propellers 
    tau = (bz * c_m* (w_prop1**2 - w_prop2**2 + w_prop3**2 - w_prop4**2) + 
        by*c_t*x_prop*(-w_prop1**2 - w_prop2**2 + w_prop3**2 + w_prop4**2) + 
        bx*c_t*y_prop*(w_prop1**2 - w_prop2**2 - w_prop3**2 + w_prop4**2))
    
    #derivative of angulaf velocity
    w_dot = np.linalg.inv(I) @ (np.cross((I @ w),w) + tau + G  )
    
    #derivative of orientation angles
    euler_angles_dot =  W1 @ w
    
    #stitch all state derivatives in one vector 
    output = np.hstack((p_dot,v_dot,euler_angles_dot,w_dot))
    
    return  output







"""test section"""
altitude_PID = PID(1,0,0)
time_array = np.linspace(0,20,100)
dt = time_array[1] - time_array[0]
output_array = []
pid_output_array = []
test_state = np.zeros(12)
test_inputs = np.array([0,0,0,0]) 

e_i = 0
e = 0
e_d = 0

set_altitude = 100 # desired altitude


for t in time_array:
    #e = set_altitude - test_state[2]
    #e_i = e*dt # integral error
    d = derivative(test_state,np.ones(4)*355)
    output_array.append(test_state[2])
    test_state = test_state + d*dt
    #e_d = ((set_altitude - test_state[2]) - e)/dt
    #test_inputs = np.ones(4) * altitude_PID.output(e,e_i,e_d)
    #pid_output_array.append(altitude_PID.output(e,e_i,e_d))
    
plt.plot(time_array, output_array)    
print(pid_output_array)

    
    
    
    