# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 12:52:44 2021

PID setup testing 

"""
import numpy as np
import pickle

class PID:
    def __init__(self,gains,sat_min,sat_max):
        self.Kp = gains[0]
        self.Ki = gains[1]
        self.Kd = gains[2]
        self.sat_max = sat_max
        self.sat_min = sat_min
        self.integral = 0
    def output(self,e,e_i,e_d):
        output = self.Kp*e + self.Kd * e_d + self.Ki * self.integral
        if output  >= self.sat_max:
            return self.sat_max
        elif output  < self.sat_min:
            return self.sat_min
        else:
            self.integral += e_i # prevent overblown integral
            return output

class Controller:
    def __init__(self,gains):
        self.gains = gains
        self.time = [0] # keep track of recorded time
        self.state_measurements = [np.array([0,0,0,0,0,0])]# keep track of previous measurements to determine error
        self.error_measurements = [np.array([0,0,0,0,0,0])]
        
        self.Roll_PID = PID(self.gains[0:3],-0.1,0.1)
        self.Pitch_PID = PID(self.gains[3:6],-0.1,0.1)
        self.Yaw_PID = PID(self.gains[6:9],-0.1,0.1)
        
        self.X_PID = PID(self.gains[9:12],-0.5,0.5)
        self.Y_PID = PID(self.gains[12:15],-0.5,0.5)
        self.Z_PID = PID(self.gains[15:18],0,0.7)
       

        
    def update(self,state_measurement,reference,t):
        dt = t - self.time[-1]
        self.time.append(t)        
        
        roll = state_measurement[3]
        pitch = state_measurement[4]
        yaw = state_measurement[5]
        
        pos_error = reference[0:2] - state_measurement[0:2] #position error in earth frame
        
        
        
        
        #transformation from earth to body frame. 
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
        
        
        #W = np.array([[ np.cos(yaw), np.sin(yaw)], 
         #             [-np.sin(yaw), np.cos(yaw)],])
        #position error in body reference frame
        pos_error = (R_b_E @ np.hstack((pos_error,np.array([0]))))[:-1]
        
        e_x_p = pos_error[0]
        e_y_p = pos_error[1]
        
        e_x_i = e_x_p*dt
        e_y_i = e_y_p*dt

        e_x_d = (e_x_p - self.error_measurements[-1][0])/dt
        e_y_d = (e_y_p - self.error_measurements[-1][1])/dt
        #print(e_x_d,"  ",e_y_d)
        desired_pitch = self.X_PID.output(e_x_p,e_x_i,e_x_d)
        desired_roll = self.Y_PID.output(-e_y_p,-e_y_i,-e_y_d)
        
        inner_reference = np.array([reference[2],desired_roll,desired_pitch,reference[3]])
        
        # inner PID 
        #optimize this line later
        inner_state_measurement = state_measurement[2:]
        
        e_p = inner_reference - inner_state_measurement
        previous_inner_error = self.error_measurements[-1][2:]
        e_i = e_p * dt
        e_d = (e_p - previous_inner_error)/dt 

            
        Z_PID_signal = np.ones(4) * self.Z_PID.output(e_p[0],e_i[0],e_d[0])
        Roll_PID_signal = np.array([1,-1,-1,1]) * self.Roll_PID.output(e_p[1],e_i[1],e_d[1])
        Pitch_PID_signal = np.array([-1,-1,1,1]) * self.Pitch_PID.output(e_p[2],e_i[2],e_d[2])
        Yaw_PID_signal = np.array([1,-1,1,-1]) * self.Yaw_PID.output(e_p[3],e_i[3],e_d[3])
        
        
        self.state_measurements.append(state_measurement)
        self.error_measurements.append(np.hstack((pos_error,e_p)))
        
        outputs = Z_PID_signal + Roll_PID_signal + Pitch_PID_signal + Yaw_PID_signal
        #print(outputs)
        return outputs
        

class Data:
    def __init__(self):
        self.time = []
        self.states = []
        self.derivatives = []
        self.control_inputs = []
        self.references = []
        
    def update(self,state,derivative,control_input,reference,t): # append 
        self.states.append(state)
        self.control_inputs.append(control_input)
        self.time.append(t)
        self.derivatives.append(derivative)
        self.references.append(reference)
    def provide(self,state_name): # provide a state variablle 
        state_names = ["x","y","z","v_x","v_y","v_z","roll","pitch","yaw","w_x","w_y","w_z"]
        control_input_names = ["t1","t1","t1","t1"]
        profile_names = ["X_profile" , "Y_profile" , "Z_profile" , "Yaw_profile"]
        if state_name in state_names:
            i = state_names.index(state_name)
            A = np.array(self.states)
        elif type(state_name) == type(1):
            i = state_name
            A = np.array(self.states)
        elif state_name in control_input_names:
            i = control_input_names.index(state_name)
            A = np.array(self.control_inputs)
        elif state_name in profile_names:
            i = profile_names.index(state_name)
            A = np.array(self.references)
        else :
            raise Exception("BETTER LUCK NEXT TIME!")
            
        B = np.transpose(A)
        return B[i], np.array(self.time), state_names[i]
        
    def measured_state(self):
        a = self.states[-1][6:9]
        h = self.states[-1][2]
        #print(np.hstack((a,[h])))
        return np.hstack((a,[h]))
    
    
    def save(self,filename):
        outfile = open(filename,'wb')
        pickle.dump([self.time,self.states,self.derivatives,self.control_inputs,self.references],outfile)
        outfile.close()
        
    def load(self,filename):
        infile = open(filename,'rb')
        a = pickle.load(infile)
        self.time = a[0]
        self.states = a[1]
        self.derivatives = a[2]
        self.control_inputs = a[3]
        self.references = a[4]
        infile.close()
    
"""       
array = np.array([np.random.rand(12),np.random.rand(12)*10,np.random.rand(12)*100,np.random.rand(12)*1000])
array2 = np.array([np.random.rand(4),np.random.rand(4)*10,np.random.rand(4)*100,np.random.rand(4)*1000])
test_data = Data()
test_data.states = array
test_data.control_inputs = array2
print(test_data.provide("w1"))
"""    
        
        
        
        
        
        
        