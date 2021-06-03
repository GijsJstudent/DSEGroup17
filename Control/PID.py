# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 12:52:44 2021

PID setup testing 
"""
import numpy as np

class PID:
    def __init__(self,Kp,Ki,Kd,B,saturation):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.saturation = saturation
        self.B = B
        self.integral = 0
    def output(self,e,e_i,e_d):
        output = self.Kp*e + self.Kd * e_d + self.Ki * self.integral
        if output + self.B >= self.saturation:
            return self.saturation
        elif output + self.B < 0:
            return 0
        else:
            self.integral += e_i # prevent overblown integral
            return output + self.B

class Data:
    def __init__(self):
        self.time = []
        self.states = []
        self.control_inputs = []
        
    def append(self,state,control_input,t): # append 
        self.states.append(state)
        self.control_inputs.append(control_input)
        self.time.append(t)
        
    def provide(self,state_name): # provide a state variablle 
        state_names = ["x","y","z","v_x","v_y","v_z","roll","pitch","yaw","w_x","w_y","w_z"]
        control_input_names = ["w1","w2","w3","w4"]
        if state_name in state_names:
            i = state_names.index(state_name)
            A = np.array(self.states)
        
        elif state_name in control_input_names:
            i = control_input_names.index(state_name)
            A = np.array(self.control_inputs)
        else :
            raise Exception("BETTER LUCK NET TIME!")
            
        B = np.transpose(A)
        return B[i], np.array(self.time)
        
    
        
array = np.array([np.random.rand(12),np.random.rand(12)*10,np.random.rand(12)*100,np.random.rand(12)*1000])
array2 = np.array([np.random.rand(4),np.random.rand(4)*10,np.random.rand(4)*100,np.random.rand(4)*1000])
test_data = Data()
test_data.states = array
test_data.control_inputs = array2
print(test_data.provide("w1"))
        
        
        
        
        
        
        
        