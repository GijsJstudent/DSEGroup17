# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 12:52:44 2021

PID setup testing 
"""

class PID:
    def __init__(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.integral = 0
    def output(self,e,e_i,e_d):
        output = self.Kp*e + self.Kd * e_d * self.Ki * self.integral
        if output >= 1:
            return 1
     
        else:
            self.integral += e_i # prevent overblown integral
            return output
