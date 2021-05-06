import numpy as np
from math import *
import scipy.integrate as integrate




def lift(rho,S,Cla,Cl0,alpha,rpm,R):
    Ltot = np.zeros(len(alpha))
    Lerror = np.zeros(len(alpha))
    for i in range(len(alpha)):
        Ltot[i],Lerror[i], = integrate.quad(lambda x: 0.5*rho*S*(Cla+Cl0)*alpha[i]*(rpm*2*np.pi/60*x)**2,0,R)
    return Ltot, Lerror

rho = 1.225
S = 1
Cla = 0.0971444
Cl0 = 0
alpha = np.array([0,2])
rpm = 1000
R = 0.05

print(lift(rho,S,Cla,Cl0,alpha,rpm,R))