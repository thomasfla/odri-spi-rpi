#!/usr/bin/python3
#Thomas Flayols - feb 2022
import time
from IPython import embed
from odri_spi_rpi import *
import time
dt = 0.001                         
ud = SPIuDriver()
ud.transfer()
t = time.perf_counter()
goalPosition = 0.0
while True:
    goalPosition +=0.001
    ud.transfer() #transfer 
    print (0.1*(goalPosition-ud.position0))
    ud.refCurrent0 = 10.0*(goalPosition-ud.position0)-0.2*ud.velocity0
    print (ud.refCurrent0 )
    #wait for next control cycle
    t +=dt 
    while(time.perf_counter()-t<dt):
        pass
