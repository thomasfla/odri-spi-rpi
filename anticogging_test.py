#!/usr/bin/python3
#Thomas Flayols - feb 2022
import time
from IPython import embed
from odri_spi_rpi import *
import numpy as np
from numpy import exp
dt = 0.001                         
ud = SPIuDriver(absolutePositionMode=True)
ud.transfer()
t = time.perf_counter()

#get cooging map fft data
data=np.load("cogging_fft.npz")
n = int(data["cogging_n"])
A = data["cogging_coef"]
freq = data["cogging_freq"]

def cogging(position):
    x=0
    m = position*n/(2.*pi)
    l = len(A)
    for k in range(l):
        x  +=  (A[k] * exp(freq[k]*m)).real
    return x

i=0
while (1):
    ud.transfer() #transfer 
    i+=1
    if i%2000 > 1000 :
        ud.refCurrent0 = cogging(ud.position0) * 1
    else:
        ud.refCurrent0 = 0
    #wait for next control cycle
    t +=dt 
    print (f"p={ud.position0} \t cog={cogging(ud.position0)}")
    
    while(time.perf_counter()-t<dt):
        pass
        
