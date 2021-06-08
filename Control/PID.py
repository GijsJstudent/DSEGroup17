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
        self.state_measurements = [np.array([0,0,0,0])]# keep track of previous measurements to determine error
        
        self.Roll_PID = PID(self.gains[0:3],-0.1,0.1)
        self.Pitch_PID = PID(self.gains[3:6],-0.1,0.1)
        self.Yaw_PID = PID(self.gains[6:9],-0.1,0.1)
        
        self.X_PID = PID(self.gains[9:12],-0.1,0.1)
        self.Y_PID = PID(self.gains[12:15],-0.1,0.1)
        self.Z_PID = PID(self.gains[15:18],0,0.7)
       

        
    def update(self,state_measurement,desired_state,t):
        
        dt = t-self.time[-1]
        self.time.append(t)
        
        e_p = desired_state - state_measurement
        e_i = e_p * dt
        if not dt == 0:
            e_d = (e_p - (desired_state - self.state_measurements[-1]))/dt 
        else:
            e_d = np.zeros(4)
            print("zeros")
        
        Roll_PID_signal = np.array([1,-1,-1,1]) * self.Roll_PID.output(e_p[0],e_i[0],e_d[0])
        Pitch_PID_signal = np.array([-1,-1,1,1]) * self.Pitch_PID.output(e_p[1],e_i[1],e_d[1])
        Yaw_PID_signal = np.array([1,-1,1,-1]) * self.Yaw_PID.output(e_p[2],e_i[2],e_d[2])
        Altitude_PID_signal = np.ones(4) * self.Z_PID.output(e_p[3],e_i[3],e_d[3])
        
        self.state_measurements.append(state_measurement)
        
        
        outputs = Altitude_PID_signal + Roll_PID_signal + Pitch_PID_signal + Yaw_PID_signal
        
        return outputs
        

class Data:
    def __init__(self):
        self.time = []
        self.states = []
        self.derivatives = []
        self.control_inputs = []
        
    def update(self,state,derivative,control_input,t): # append 
        self.states.append(state)
        self.control_inputs.append(control_input)
        self.time.append(t)
        self.derivatives.append(derivative)
    def provide(self,state_name): # provide a state variablle 
        state_names = ["x","y","z","v_x","v_y","v_z","roll","pitch","yaw","w_x","w_y","w_z"]
        control_input_names = ["t1","t1","t1","t1"]
        if state_name in state_names:
            i = state_names.index(state_name)
            A = np.array(self.states)
        elif type(state_name) == type(1):
            i = state_name
            A = np.array(self.states)
        elif state_name in control_input_names:
            i = control_input_names.index(state_name)
            A = np.array(self.control_inputs)
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
        pickle.dump([self.time,self.states,self.derivatives,self.control_inputs],outfile)
        outfile.close()
        
    def load(self,filename):
        infile = open(filename,'rb')
        a = pickle.load(infile)
        self.time = a[0]
        self.states = a[1]
        self.derivatives = a[2]
        self.control_inputs = a[3]
        infile.close()
    
"""       
array = np.array([np.random.rand(12),np.random.rand(12)*10,np.random.rand(12)*100,np.random.rand(12)*1000])
array2 = np.array([np.random.rand(4),np.random.rand(4)*10,np.random.rand(4)*100,np.random.rand(4)*1000])
test_data = Data()
test_data.states = array
test_data.control_inputs = array2
print(test_data.provide("w1"))
"""    
        
        
        
        
        
        
        