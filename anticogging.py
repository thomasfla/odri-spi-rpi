#!/usr/bin/python3
#Thomas Flayols - feb 2022
import time
from IPython import embed
from odri_spi_rpi import *
import numpy as np
dt = 0.001                         
ud = SPIuDriver(absolutePositionMode=True)
ud.transfer()
t = time.perf_counter()
goalPosition = 0.0
N=500000
i=0
log_p = np.empty(N) * np.nan
log_pref = np.empty(N) * np.nan
log_i = np.empty(N) * np.nan
log_iref = np.empty(N) * np.nan
log_i_filt = np.empty(N) * np.nan
log_i_filt[-1] = 0
coggingTorque = []
coggingPosition = []

myPID = PID(Kp = 15.0, Ki = 800.0, Kd = 0*0.10, sat=5)
ud.kd0= 0.2 #set the kd in low level for higher bandwith
a=0.99
while (goalPosition<2*pi and i<N):
    if (i%200==0):
        coggingTorque.append(log_i_filt[i-1])
        coggingPosition.append(log_pref[i-1])
        goalPosition +=0.002
    ud.transfer() #transfer 
    ud.refCurrent0 = myPID.compute(ud.position0, ud.velocity0,goalPosition,0.0 )
    log_p[i]=ud.position0
    log_pref[i]=goalPosition
    log_i[i]=ud.current0
    log_iref[i]=ud.refCurrent0
    log_i_filt[i]=log_i_filt[i-1] * a + (1-a) * log_i[i]
    i+=1
    #wait for next control cycle
    t +=dt 
    while(time.perf_counter()-t<dt):
        pass
        

import matplotlib.pyplot as plt
plt.plot(log_p)
plt.plot(log_pref)
plt.figure()
plt.plot(log_i)
plt.plot(log_iref)
plt.plot(log_i_filt)
plt.figure()
plt.plot(log_iref,log_p)
plt.figure()
plt.plot(coggingPosition,coggingTorque,'x')
plt.show()
pos=np.array(coggingPosition)
cur=np.array(coggingTorque)
np.savez("cogging_map.npz",pos,cur)



embed()
