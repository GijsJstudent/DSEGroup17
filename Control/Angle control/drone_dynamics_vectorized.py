# -*- coding: utf-8 -*-
"""
Created on Wed Jun  2 14:12:12 2021
"""

# -*- coding: utf-8 -*-
"""
Created on Sun May 30 21:06:41 2021
3D drone dynamics v1.0
"""
import numpy as np
import matplotlib.pyplot as plt 
from PID import PID ,Data, Controller
import pickle
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
    
    F_thrust = R_E_b @ np.array([0,0,c_t*(np.linalg.norm(scaled_inputs))**2]) # thrust force in E frame
    #F_thrust = R_E_b @ np.array([0,0,c_t*np.sum(np.square(scaled_inputs))])
    
    
    F_a = (-1*v) * np.linalg.norm(v) * 0.5 * rho * C_d * S # aerodynamic force
    #print(np.linalg.norm(F_a))
    #print(np.linalg.norm(v))
    v_dot = (1/M) * (F_g + F_a + F_thrust) # derivative of velocity in E frame in vector form
    #print(F_g)
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

p = 0.8
d = 0.1
it = 0

#altitude_PID = PID([0.8,0.1,0.2],0,0.7)
#roll_PID = PID([p,it,d],-0.1,0.1)
#pitch_PID = PID([p,it,d],-0.1,0.1)
#yaw_PID = PID([0.03,0,0.018],-0.1,0.1)

# initialize gains of the controller roll,pitch,yaw,x,y,z
test_controller = Controller([p,it,d,p,it,d,0.08,0,0.018,0,0,0,0,0,0,0.8,0.1,0.8])

time_array = np.linspace(0,10,10000)
dt = time_array[1] - time_array[0]

state = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
d= np.zeros(12)
control_input = np.array([0,0,0,0]) 

# define mission flight profile
Altitude_profile = np.ones(len(time_array)) * 1200
Yaw_profile = np.ones(len(time_array)) * -0.3
Roll_profile = (np.sin(time_array/5)) * 0
Pitch_profile = (np.sin(time_array/5)) * 0
Roll_profile = np.ones(len(time_array)) * -0.5
Pitch_profile = np.ones(len(time_array)) * 0.5

print(len(Yaw_profile),len(Roll_profile),len(Pitch_profile))

flight_data = Data() # initialize solution array
flight_data.update(np.zeros(12),np.zeros(12),np.zeros(4),0)# set initial conditions

""" main loop"""
for t, i in zip(time_array[1:],range(1,len(time_array))):
    
    set_altitude = Altitude_profile[i] # reference altitude at this time
    set_yaw = Yaw_profile[i]        #reference yaw at this time
    set_pitch = Pitch_profile[i]    #            
    set_roll = Roll_profile[i]      #
    desired_state = np.array([set_roll,set_pitch,set_yaw,set_altitude])
    
    current_state = np.array([state[6],state[7],state[8],state[2]]) # roll pitch yaw and altitude at this moment
    
    control_input = test_controller.update(current_state,desired_state,t) # calculate the inputs to the rotors
    #print(control_input)
    
    d = derivative(state,control_input) # derivative of the state 
    state = state + d*dt
    flight_data.update(state,d,control_input,t)
    
flight_data.save("flight_data")
flight_data2 = Data()
flight_data2.load("flight_data")

"""Plot the results"""

fig, axs = plt.subplots(4, 3,figsize = (14,7),sharex=True)
time_array = flight_data2.provide(1)[1]
print(len(time_array))
print(len(flight_data2.provide(1)[0]))
for ax,i in zip(axs.flat,range(12)):
    ax.plot(time_array,flight_data2.provide(i)[0])
    
    if i == 2:
        ax.plot(time_array,Altitude_profile)
        
    if i == 6:
        ax.plot(time_array,Roll_profile)
        
    if i == 7:
        ax.plot(time_array,Pitch_profile)
        
    if i == 8:
        ax.plot(time_array,Yaw_profile)
    
    ax.set( ylabel=flight_data2.provide(i)[2])
    
    
plt.show()


