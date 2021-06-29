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



def simulate_drone(pid_gains, time_array ,Command_matrix,wind_disturbance = None):
    
    # defining drone parameters     
    # unit vectors in x,y,z directions (might be useful later)
    bx= np.array([1,0,0])
    by= np.array([0,1,0])
    bz= np.array([0,0,1])
    
    I = [[30262613*10**-9, 0,          0       ], # drone moment of inertia 
         [0,          19302074*10**-9, 0       ],
         [0,          0,          40446772*10**-9]]
    # constants identification 
    I_prop = 7.74* (10**-5)# propeller moment of inertia 
    # w_prop_max = 1000 # maximum angular velocity of propellers
    M = 2.11 # [kg] mass of the drone 
    x_prop = 0.178 #[m] x distance from c.g. to propeller
    y_prop = 0.178 #[m] y distance from c.g. to propeller
    g = 9.81 # gravitational constant 
    c_m = 4.33*(10**-6) # torque coefficient of propeller T = c_m * w^2
    c_t = 1*(10**-4) # thrust coefficient of propeller F = c_m * w^2
    S = 0.024 # reference area of the drone 
    C_d = 1.21 #drag coefficient
    rho = 1.225 # density of air 
    # F_max = c_t*w_prop_max**2
    # T_W_ratio = F_max/(M*g)
    T_W_ratio = 3
    motors = 4 # Number of motors
    F_max = T_W_ratio*g*M/motors
    w_prop_max = (F_max)**0.5 / c_t
    print(f"T/W ratio: {T_W_ratio} [-]\nMax total thrust: {F_max*4} [N]")
    
    # function that calculates derivative of the state from state and external forces
    def derivative(state,inputs,wind_speed):
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
        
        F_wind = wind_speed * np.linalg.norm(wind_speed) * 0.5 * rho * C_d * S # wind force in E frame 
        
        F_thrust = R_E_b @ np.array([0,0,c_t*np.sum(np.square(scaled_inputs))]) # thrust force in E frame
        
        F_a = (-1*v) * np.linalg.norm(v) * 0.5 * rho * C_d* S # aerodynamic force
        #F_a = np.zeros(3)
        v_dot = (1/M) * (F_g + F_a + F_thrust + F_wind) # derivative of velocity in E frame in vector form
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
        w_dot = np.linalg.inv(I) @ (np.cross((I @ w),w) + tau + G)
        #derivative of orientation angles
        euler_angles_dot =  W2 @ w
        
        #stitch all state derivatives in one vector 
        output = np.hstack((p_dot,v_dot,euler_angles_dot,w_dot))
        
        return  output

    
    
    
    # initialize gains of the controller roll,pitch,yaw,x,y,z
    test_controller = Controller(pid_gains)
    


    dt = time_array[1] - time_array[0]
    #print(f"SIMULATION TIME: {round(len(time_array)*dt,2)}\nTIMESTEP: {dt}")

    state = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
    d= np.zeros(12)
    control_input = np.array([0,0,0,0]) 
    
    
    flight_data = Data() # initialize solution array
    flight_data.update(np.zeros(12),np.zeros(12),np.zeros(4),Command_matrix[0],np.zeros(3),0)# set initial conditions
    
    
    for i,t, reference, wind_speed in zip(range(len(time_array[1:])),time_array[1:],Command_matrix[1:],wind_disturbance[1:]):
        
        #capture all states relevant to pid controller (x y z roll pitch yaw)
        current_state = np.hstack((state[0:3],state[6:9]))
        
        # calculate the inputs to the rotors
        if i%2 == 0:
            control_input = test_controller.update(current_state,reference,t) 
        
        # derivative of the state 
        d = derivative(state,control_input,wind_speed) 
        
        # first order technique: 
        state = state + d*dt
    
        #update the flight data with new state 
        flight_data.update(state,d,control_input,reference,wind_speed,t)
        
    return flight_data






""" Main program """   
 

p_rp = 0.8 #proportional gain for roll and pitch controllers
d_rp = 0.1 #proportional gain for roll and pitch controllers

it_rp = 0 #proportional gain for roll and pitch controllers
    
p_xy = 0.1 #proportional gain for x and y controllers
d_xy = 0.2 #proportional gain for x and y controllers
it_xy = 0.02#proportional gain for x and y controllers
         

pid_gains = [p_rp,it_rp,d_rp,p_rp,it_rp,d_rp,0.08,0,0.018,p_xy,it_xy,d_xy,p_xy,it_xy,d_xy,0.8,0.1,1.7]    
"""
pid_gains = [ 9.39362754e-01 ,-1.25274567e-03 , 1.03179595e-01 , 6.05617003e-01,
 -2.47850329e-06,  1.21172893e-01, 8.35487397e-02 ,-8.51006653e-04,
  1.76026504e-02 , 8.19871955e-02, -5.20418186e-04,  2.39828488e-01,
  1.34284217e-01 ,-6.24434720e-04 , 2.40449375e-01 , 7.90964222e-01,
  1.00254316e-01 , 7.49856318e-01]
"""
max_time = 40
dt = 0.001
N_time = int(max_time/dt)
time_stamps = 20000
time_array = np.linspace(0,max_time,N_time)     

 

# spiral with 0 wind and heading following
"""
max_time = 20
dt = 0.001
N_time = int(max_time/dt)

time_array = np.linspace(0,max_time,N_time)     
Z_profile = np.ones(len(time_array)) * 10
Theta = np.linspace(0,3.14*4,N_time)
R = np.linspace(0,10,N_time)
X_profile = np.cos(Theta) * R
Y_profile = np.sin(Theta) * R
X_1_profile = np.hstack((X_profile[1:],X_profile[-1]))
Y_1_profile = np.hstack((Y_profile[1:],Y_profile[-1]))
D_X = (X_profile - X_1_profile)/dt
D_Y = (Y_profile - Y_1_profile)/dt

Yaw_profile = np.arctan2(D_Y,D_X)
new_yaw = [0,0]
suma = 0
for i in range(len(Yaw_profile)-2):
    if Yaw_profile[i]>Yaw_profile[i+2]:
        suma += np.pi
    new_yaw.append(suma+Yaw_profile[i])
Yaw_profile = np.array(new_yaw)   +  np.pi/2      
#Yaw_profile = np.ones(len(time_array)) * 0
x_wind = np.ones(len(time_array)) * 0
y_wind = np.ones(len(time_array)) * 0
z_wind = np.ones(len(time_array)) * 0  
"""

# fancy trajectory with 0 wind
"""
Z_profile = np.linspace(0,20, len(time_array))
R = (-1/25)* Z_profile**2 + 4 * Z_profile
Theta = np.cos(time_array/5)
Y_profile = R*np.cos(Theta)
X_profile = R*np.sin(Theta)
Yaw_profile = np.ones(len(time_array)) * 0
x_wind = np.ones(len(time_array)) * 0
y_wind = np.ones(len(time_array)) * 0
z_wind = np.ones(len(time_array)) * 0
"""

# helical trajectory with 0 wind
"""
X_profile = np.linspace(0,20, len(time_array))
Theta = np.linspace(0,20, len(time_array))
Y_profile = 10*np.cos(Theta)
Z_profile = 10*np.sin(Theta) + 10
Yaw_profile = np.linspace(0,20, len(time_array))*0

x_wind = np.ones(len(time_array)) * 0
y_wind = np.ones(len(time_array)) * 0
z_wind = np.ones(len(time_array)) * 0
"""

"""
# lorents attractor:
max_time = 300
dt = 0.001
N_time = int(max_time/dt)
time_stamps = 20000
time_array = np.linspace(0,max_time,N_time) 
 
def Der(vec,r,b,sigma):
    x = vec[0]
    y = vec[1]
    z = vec[2]
    x_dot = sigma*(y-x)
    y_dot = x*(r-z)-y
    z_dot = x*y - b*z
    return(np.array([x_dot,y_dot,z_dot]))*0.03

r = 28
b = 8/3
sigma = 5
vec = np.array([5,5,5])

solution = []

for t ,i in zip(time_array, range(len(time_array))):
    vec_dot = Der(vec,r,b,sigma)
    vec = vec + vec_dot*dt
    solution.append(vec)

solution = np.array(solution).transpose()
X_profile = solution[0]
Y_profile = solution[1]
Z_profile = solution[2]
Yaw_profile = np.ones(len(time_array)) * 0
x_wind = np.ones(len(time_array)) * 0
y_wind = np.ones(len(time_array)) * 0
z_wind = np.ones(len(time_array)) * 0
"""

# position keeping with wind gusts 
max_time = 40
dt = 0.001
N_time = int(max_time/dt)

time_array = np.linspace(0,max_time,N_time)
  
Z_profile = np.linspace(0,5, len(time_array))
#Z_profile = np.ones(len(time_array)) * 0
Y_profile = np.linspace(0,0, len(time_array))
X_profile = np.linspace(0,0, len(time_array))
Yaw_profile = np.ones(len(time_array)) * 0

x_wind = np.ones(len(time_array)) * 0
x_wind[int(max_time/(5*dt)):]  = np.ones(len(x_wind[int(max_time/(5*dt)):])) * 15
y_wind = np.ones(len(time_array)) * 0
z_wind = np.ones(len(time_array)) * 0


"""
Z_profile = np.ones(len(time_array)) * 1000
Y_profile = np.ones(len(time_array)) * 100
X_profile = np.ones(len(time_array)) * 100
Yaw_profile = np.ones(len(time_array)) * 0
x_wind = np.ones(len(time_array)) * 0
y_wind = np.ones(len(time_array)) * 0
z_wind = np.ones(len(time_array)) * 0

"""


Command_matrix = np.vstack((X_profile,Y_profile,Z_profile,Yaw_profile))
Command_matrix = np.transpose(Command_matrix)
Wind_gusts = np.vstack((x_wind,y_wind,z_wind))
Wind_gusts = np.transpose(Wind_gusts)
# print(Command_matrix[0])    
   
flight_data = simulate_drone(pid_gains, time_array ,Command_matrix,Wind_gusts)

flight_data.save("flight_data")    


fig, axs = plt.subplots(4, 3,figsize = (14,7),sharex=True)
time_array = flight_data.provide(1)[1]

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
    ax.set( xlabel="time")
    
plt.show()



fig, axs = plt.subplots(5)

axs[0].plot(time_array, x_wind, label = "Wind speed in x direction")
axs[0].set( ylabel="wind speed [m/s]")

axs[1].plot(time_array, flight_data.provide("x")[0],label = "Drone x coordinate" )
axs[1].plot(time_array, X_profile, label = "Desired x coordinate" )
axs[1].set( ylabel="[m]")




axs[2].plot(time_array, flight_data.provide("v_x")[0],label = "Drone velocity in x direction")
axs[2].set( ylabel="[m/s]")

axs[3].plot(time_array, flight_data.provide("pitch")[0],label = "Drone pitch angle")
axs[3].set( ylabel="[rad]")

axs[4].plot(time_array, flight_data.provide("w_y")[0],label = "Drone angular velocity around y axis")
axs[4].set( ylabel="[rad/s]", xlabel= "[time]")
axs[0].legend()
axs[1].legend()
axs[2].legend()
axs[3].legend()
axs[4].legend()
#axs[1].legend()
plt.show()



