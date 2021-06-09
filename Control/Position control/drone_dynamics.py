# -*- coding: utf-8 -*-
"""
Created on Sun May 30 21:06:41 2021

3D drone dynamics v1.0
"""
import numpy as np
import matplotlib.pyplot as plt 
from PID import PID ,Data
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
C_d = 0.3 #drag coefficient
rho = 1.225 # density of air 
F_max = c_t*w_prop_max**2
T_W_ratio = F_max/(M*g)


# function that calculates derivative of the state from state and external forces
def derivative(state,inputs):
# state and inputs are lists with length 12 and 4, next lines unpack them
    x,y,z,v_x,v_y,v_z = state[0:6]
    roll, pitch, yaw, w_x, w_y, w_z = state[6:12]
    
    #inputs are numbers from 0 - 1 which are thrust setting  
    inputs[inputs < 0] = 0
    scaled_inputs = (inputs* (F_max/c_t))**0.5 # convert thrust setting to rpm 
    w_prop1, w_prop2, w_prop3, w_prop4 = scaled_inputs 
    #print(w_prop1)
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
    
    #R_b_E = R_b_2 @ (R_2_1 @ R_1_E) #transformation from E frame to b frame
    
    R_E_1 = np.linalg.inv(R_1_E)
    R_1_2 = np.linalg.inv(R_2_1)
    R_2_b = np.linalg.inv(R_b_2)
    
    R_E_b = R_E_1 @ (R_1_2 @ R_2_b) # transformation from b frame to E frame
    
    
    W1 = np.linalg.inv(np.array([[1,  0           , -np.sin(pitch)             ],
                                [0,  np.cos(roll),  np.cos(pitch)*np.sin(roll)],
                                [0, -np.sin(roll),  np.cos(pitch)*np.cos(roll)]]))
    
    W2 = np.nan_to_num(np.array([[1,  np.tan(pitch)*np.sin(roll),  np.tan(pitch)*np.cos(roll)],
                   [0,  np.cos(roll)              , -np.sin(roll)              ],
                   [0,  np.sin(roll)/np.cos(pitch),  np.cos(roll)/np.cos(pitch)]]))
    
    F_g = np.array([0,0,-M*g]) #force of gravity in E frame
    
    F_thrust = R_E_b @ np.array([0,0,c_t*np.sum(np.square(scaled_inputs))]) # thrust force in E frame
    
    F_a = (-1*v) * np.linalg.norm(v) * 0.5 * rho * C_d # aerodynamic force
    #F_a = np.zeros(3)
    v_dot = (1/M) * (F_g + F_a + F_thrust) # derivative of velocity in E frame in vector form
    """
    if np.isnan(v_dot[1]):
        print("v_dot is nan")
        print("F_thrust: ",F_thrust)
        print(R_E_b)
        print("thrust in body frame: " , np.array([0,0,c_t*np.sum(np.square(scaled_inputs))]))
        print("scaled_inputs: ", scaled_inputs)
        print("inputs: ", inputs)
    """
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
    euler_angles_dot =  W2 @ w
    
    #stitch all state derivatives in one vector 
    output = np.hstack((p_dot,v_dot,euler_angles_dot,w_dot))
    
    return  output



"""test section"""
print("T/W: ",T_W_ratio, "F_max: ",F_max)

altitude_PID = PID([0.8,0.1,0.2],0,0.7)
p = 0.1
d = 0.1
it = 0
roll_PID = PID([p,it,d],-0.1,0.1)
pitch_PID = PID([p,it,d],-0.1,0.1)
yaw_PID = PID([0.03,0,0.018],-0.1,0.1)

time_array = np.linspace(0,4,2000)
dt = time_array[1] - time_array[0]
state = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
inputs = np.array([0,0,0,0]) 
e_i_h = 0
e_h = 0
e_d_h = 0

e_i_yaw = 0
e_yaw = 0
e_d_yaw = 0

e_i_roll = 0
e_roll = 0
e_d_roll = 0

e_i_pitch = 0
e_pitch = 0
e_d_pitch = 0


Altitude_profile = np.ones(len(time_array)) * 5
Yaw_profile = np.ones(len(time_array)) * 0
Roll_profile = (np.sin(time_array/5)) * 0
Pitch_profile = (np.sin(time_array/5)) * 0
Roll_profile = np.ones(len(time_array)) * 0.1
Pitch_profile = np.ones(len(time_array)) * 0.1


flight_data = Data() # initialize flight data collector

""" main loop"""
for t, i in zip(time_array,range(len(time_array))):
    set_altitude = Altitude_profile[i] # reference altitude at this time
    set_yaw = Yaw_profile[i]        #reference yaw at this time
    set_pitch = Pitch_profile[i]    #            
    set_roll = Roll_profile[i]      #
    
    # error between set values and actual values
    e_h   = set_altitude - state[2]
    e_roll = set_roll - state[6]
    e_pitch = set_pitch - state[7]
    e_yaw = set_yaw - state[8]
    
    #update state
    d = derivative(state,inputs)
    state = state + d*dt
    
    e_i_h = e_h*dt # integral error of altitude
    e_i_roll = e_roll*dt
    e_i_pitch = e_pitch*dt
    e_i_yaw = e_yaw*dt # integral error of yaw
    
    e_d_h = ((set_altitude - state[2]) - e_h)/dt
    e_d_roll = ((set_roll - state[6]) - e_roll)/dt
    e_d_pitch = ((set_pitch - state[7]) - e_pitch)/dt
    e_d_yaw = ((set_yaw - state[8]) - e_yaw)/dt
    #print(e_d_h)
    Altitude_PID_signal = np.ones(4) * altitude_PID.output(e_h,e_i_h,e_d_h)
    Roll_PID_signal = np.array([1,-1,-1,1]) * roll_PID.output(e_roll,e_i_roll,e_d_roll)
    Pitch_PID_signal = np.array([-1,-1,1,1]) * pitch_PID.output(e_pitch,e_i_pitch,e_d_pitch)
    Yaw_PID_signal = np.array([1,-1,1,-1]) * yaw_PID.output(e_yaw,e_i_yaw,e_d_yaw)
    
    inputs = Altitude_PID_signal + Roll_PID_signal + Pitch_PID_signal + Yaw_PID_signal
    #print(inputs)
    flight_data.update(state,d,inputs,t)
    
    
"""Plot the results"""

fig, axs = plt.subplots(4, 3,figsize = (14,7),sharex=True)

for ax,i in zip(axs.flat,range(12)):
    ax.plot(time_array,flight_data.provide(i)[0])
    if i == 2:
        ax.plot(time_array,Altitude_profile)
        
    if i == 6:
        ax.plot(time_array,Roll_profile)
        
    if i == 7:
        ax.plot(time_array,Pitch_profile)
        
    if i == 8:
        ax.plot(time_array,Yaw_profile)
        
    ax.set( ylabel=flight_data.provide(i)[2])
    print(i)
    
plt.show()










"""
#print(time_array)
#print(flight_data.provide('yaw')[0][0])

fig, (ax1, ax2,ax3,ax4) = plt.subplots(4,figsize = (10,7))

ax1.plot(time_array, Altitude_profile,label = "reference alt") 
ax1.plot(time_array, flight_data.provide('z')[0],label = "flight alt") 
ax1.legend() 

ax2.plot(time_array, flight_data.provide('roll')[0],label = "roll") 
ax2.legend() 

ax3.plot(time_array, flight_data.provide('pitch')[0],label = "pitch") 
ax3.legend() 

ax4.plot(time_array, Yaw_profile,label = "reference yaw") 
ax4.plot(time_array, flight_data.provide('yaw')[0],label = "yaw") 
ax4.legend() 

plt.xlabel("time")
plt.show()   

"""
    
    
    
    