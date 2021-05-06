import numpy as np
from math import *
import scipy.integrate as integrate
import matplotlib.pyplot as plt



def liftalpha(rho,S,Cla,Cl0,alpha,rpm,R):
    Ltot = np.zeros(len(alpha))
    Lerror = np.zeros(len(alpha))
    for i in range(len(alpha)):
        Ltot[i],Lerror[i], = integrate.quad(lambda x: 0.5*rho*S*(Cla*alpha[i]+Cl0)*(rpm*2*np.pi/60*x)**2,0,R)
    return Ltot

def liftrpm(rho,S,Cla,Cl0,alpha,rpm,R):
    Ltot = np.zeros(len(rpm))
    Lerror = np.zeros(len(rpm))
    for i in range(len(rpm)):
        Ltot[i],Lerror[i], = integrate.quad(lambda x: 0.5*rho*S*(Cla*alpha+Cl0)*(rpm[i]*2*np.pi/60*x)**2,0,R)
    return Ltot

rho = 1.225
b = 0.03
Cla = 0.0971444
Cl0 = 0
alpha1 = np.arange(0,10)
alpha2 = 3
rpm1 = 25000
rpm2 = np.arange(10000,50000)
R = 0.15
S = b*R


L1 = liftalpha(rho,S,Cla,Cl0,alpha1,rpm1,R)
plt.plot(alpha1,L1)
plt.xlabel('alpha [deg]')
plt.ylabel('Lift [N]')
plt.title('Alpha against life for one blade of 25000 RPM')
plt.show()


L2 = liftrpm(rho,S,Cla,Cl0,alpha2,rpm2,R)
plt.plot(rpm2,L2)
plt.xlabel('RPM')
plt.ylabel('Lift [N]')
plt.title('Alpha against life for one blade of 3 degrees AOA')
plt.show()



