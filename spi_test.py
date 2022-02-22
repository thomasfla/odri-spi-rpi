#!/usr/bin/python3

import time
from IPython import embed
from odri_spi_rpi import *

dt = 0.1
ud = SPIuDriver()

 

t = time.perf_counter()

    
while True:
    t +=dt
    #t =time.perf_counter()  
    ud.transfer()
    while(time.perf_counter()-t<dt):
        pass

    
