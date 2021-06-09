# -*- coding: utf-8 -*-
"""

Created on Sun May 30 21:06:41 2021

3D drone dynamics v2.0

"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PID import PID ,Data, Controller
import matplotlib.animation as animation
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
    
    F_thrust = R_E_b @ np.array([0,0,c_t*np.sum(np.square(scaled_inputs))]) # thrust force in E frame
    
    F_a = (-1*v) * np.linalg.norm(v) * 0.5 * rho * C_d* S # aerodynamic force
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

p_rp = 0.8 #proportional gain for roll and pitch controllers
d_rp = 0.1 #proportional gain for roll and pitch controllers
it_rp = 0 #proportional gain for roll and pitch controllers

p_xy = 0.1 #proportional gain for x and y controllers
d_xy = 0.2 #proportional gain for x and y controllers
it_xy = 0 #proportional gain for x and y controllers

#altitude_PID = PID([0.8,0.1,0.2],0,0.7)
#roll_PID = PID([p,it,d],-0.1,0.1)
#pitch_PID = PID([p,it,d],-0.1,0.1)
#yaw_PID = PID([0.03,0,0.018],-0.1,0.1)

# initialize gains of the controller roll,pitch,yaw,x,y,z
test_controller = Controller([p_rp,it_rp,d_rp,p_rp,it_rp,d_rp,0.08,0,0.018,p_xy,it_xy,d_xy,p_xy,it_xy,d_xy,0.8,0.1,0.8])

simtime = 5
dt = 1/100

time_array = np.linspace(0,20,int(simtime/dt))
# dt = time_array[1] - time_array[0]
print(f"SIMULATION TIME: {round(len(time_array)*dt,2)}\nTIMESTEP: {dt}")

state = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
d= np.zeros(12)
control_input = np.array([0,0,0,0]) 

# define mission flight profile


X_profile = np.ones(len(time_array)) * 100
Y_profile = np.ones(len(time_array)) * 100
Z_profile = np.ones(len(time_array)) * 4


X_profile = np.sin(time_array/10) * 0
Y_profile = np.cos(time_array/10) * 5
Z_profile = np.linspace(2, 5, len(Y_profile))


Yaw_profile = np.ones(len(time_array)) * 0

Command_matrix = np.vstack((X_profile,Y_profile,Z_profile,Yaw_profile))
Command_matrix = np.transpose(Command_matrix)
# print(Command_matrix[0])


flight_data = Data() # initialize solution array
flight_data.update(np.zeros(12),np.zeros(12),np.zeros(4),Command_matrix[0],0)# set initial conditions


for t, reference in zip(time_array[1:],Command_matrix[1:]):
    
    #capture all states relevant to pid controller (x y z roll pitch yaw)
    current_state = np.hstack((state[0:3],state[6:9]))
    
    # calculate the inputs to the rotors
    control_input = test_controller.update(current_state,reference,t) 
    
    # derivative of the state 
    d = derivative(state,control_input) 
    
    # first order technique: 
    state = state + d*dt

    #update the flight data with new state 
    flight_data.update(state,d,control_input,reference,t)
    
    

    
flight_data.save("flight_data")

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
    x, y, z = calc_motor_pos(i)
    lines.append(ax.plot([tra_x[i]-x, tra_x[i]+x],[tra_y[i]-y, tra_y[i]+y],[tra_z[i]-z, tra_z[i]+z], 'c')[0])
    lines.append(ax.plot([tra_x[i]+x, tra_x[i]-x],[tra_y[i]-y, tra_y[i]+y],[tra_z[i]-z, tra_z[i]+z], 'c')[0])
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
arm = np.array([0.5, 0.5, 0]).T

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
    pos = R_E_b @ arm
    x = pos[0]
    y = pos[1]
    z = pos[2]
    print(pos)
    return x,y,z

# Generate lines
line1 = ax.plot(tra_x[0], tra_y[0], tra_z[0], 'r')[0]
line2 = ax.plot(X_profile[0], Y_profile[0], Z_profile[0], 'b')[0]
# line3 = ax.plot([tra_x[0]-0.5, tra_x[0]+0.5],[tra_y[0]-0.5, tra_y[0]+0.5],[tra_z[0], tra_z[0]], 'c')[0]
# line4 = ax.plot([tra_x[0]+0.5, tra_x[0]-0.5],[tra_y[0]-0.5, tra_y[0]+0.5],[tra_z[0], tra_z[0]], 'c')[0]
lines = [line1, line2]

line_ani = animation.FuncAnimation(fig, update_graph, int(len(time_array)/sfactor), fargs=(sfactor, lines),
                                   interval=100, blit=True)

plt.show()


# 3D plotting trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

Axes3D.plot(ax, flight_data.provide(0)[0], flight_data.provide(1)[0], flight_data.provide(2)[0])
Axes3D.plot(ax, X_profile, Y_profile, Z_profile)
plt.show()



fig, axs = plt.subplots(4, 3,figsize = (14,7),sharex=True)
time_array = flight_data.provide(1)[1]
print("time array len",len(time_array))
print("asasas",len(flight_data.provide(1)[0]))
for ax,i in zip(axs.flat,range(12)):
    ax.plot(time_array,flight_data.provide(i)[0])
    
    if i == 0:
        ax.plot(time_array,X_profile)
        
    if i == 1:
        ax.plot(time_array,Y_profile)
        
    if i == 2:
        ax.plot(time_array,Z_profile)
        
    if i == 8:
        ax.plot(time_array,Yaw_profile)
    
    ax.set( ylabel=flight_data.provide(i)[2])
    
    
plt.show()


