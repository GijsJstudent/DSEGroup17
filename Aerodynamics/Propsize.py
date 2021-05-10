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
AR = 5
Cla = 0.0971444
Cl0 = 0
R = np.arange(0.01,0.51,0.01)
b = R/AR
S = b*R

maxrpm = 50000
Ttotal = 54
nprop = 4
eff = 0.55
T = Ttotal/nprop/eff



for alpha in range(1,20,2):
    rpm = np.sqrt(T/(1/6*rho*S*(Cla*alpha+Cl0)*(2*np.pi/60)**2*R**3))
    plt.plot(rpm,R,label = 'alpha=%s'%alpha)

plt.vlines(maxrpm,0,0.50,colors='k',linestyles='dashed')
plt.legend()
plt.xlim(0,70000)
plt.ylim(0,0.50)
plt.title('RPM versus radius of propeller for %s propellers and a total thrust of %s N'%(nprop,Ttotal))
plt.xlabel('Rotations per minute of the engine')
plt.ylabel('Radius of propeller')
plt.show()





#
# L1 = liftalpha(rho,S,Cla,Cl0,alpha1,rpm1,R)
# plt.plot(alpha1,L1)
# plt.xlabel('alpha [deg]')
# plt.ylabel('Thrust [N]')
# plt.title('Alpha against life for one blade of 25000 RPM')
# plt.show()
#
#
# L2 = liftrpm(rho,S,Cla,Cl0,alpha2,rpm2,R)
# plt.plot(rpm2,L2)
# plt.xlabel('RPM')
# plt.ylabel('Thrust [N]')
# plt.title('Alpha against life for one blade of 3 degrees AOA')
# plt.show()
#


