#!/usr/bin/python3
import RPi.GPIO as GPIO
import spidev
import crcmod
import struct
import time
from IPython import embed

dt = 0.1
spi = spidev.SpiDev()
spi.open(0, 0)
spi.mode=0
spi.max_speed_hz = 80000

 
GPIO.setmode(GPIO.BCM)
GPIO.setup(25,GPIO.OUT)
GPIO.output(25,0)
t = time.perf_counter()
while True:
    t +=dt
    #t =time.perf_counter()  
    GPIO.output(25,0) 
    res = spi.xfer([0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
    print(res)
    
    GPIO.output(25,1)
    while(time.perf_counter()-t<dt):
        pass

    #embed()
