#!/usr/bin/env python3

#Thomas Flayols - feb 2022
#https://github.com/thomasfla/odri-spi-rpi
#https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_%C2%B5Driver_SPI_interface.md
import struct
import spidev
import RPi.GPIO as GPIO
import spidev
from IPython import embed
from math import pi
import time
def crc32(buf):
  crc=0xffffffff
  for val in buf:
    crc ^= val << 24
    for _ in range(8):
      crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
  return crc
  
def checkcrc(buf):
  crc = crc32(buf[:-4])
  if (crc&0xffff == buf[-4]*256+buf[-3] and (crc&0xFFFF0000)>>16 == buf[-2]*256+buf[-1] ): #todo, clean
    return True
  return False

class SPIuDriver:
  def __init__(self, waitForInit = True):
    
    #Configure CS pin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(25,GPIO.OUT)
    GPIO.output(25,0)

    #Initialise SPI
    self.spi = spidev.SpiDev()
    self.spi.open(0, 0)
    self.spi.mode=0
    self.spi.max_speed_hz = 8000000
    
    # Allocate all variables
    self.is_system_enabled = 0
    self.error_code = 0
    self.position0 = 0
    self.position1 = 0
    self.velocity0 = 0
    self.velocity1 = 0
    self.current0 = 0
    self.current1 = 0
    self.is_enabled0 = 0
    self.is_enabled1= 0
    self.is_ready0 = 0
    self.is_ready1 = 0
    self.has_index_been_detected0 = 0
    self.has_index_been_detected1 = 0
    self.index_toggle_bit0 = 0
    self.index_toggle_bit1 = 0
    

    self.refPosition0 = 0
    self.refPosition1= 0
    self.refVelocity0= 0
    self.refVelocity1= 0
    self.refCurrent0= 0
    self.refCurrent1= 0
    self.iSatCurrent0= 0
    self.iSatCurrent1= 0
    self.kp0= 0
    self.kp1= 0
    self.kd0= 0
    self.kd1= 0
  
    self.error = -1
    #wait for system enable
    if waitForInit:
      while(not self.is_ready0):
        self.transfer()
        time.sleep(0.001)
    print ("ready!")
  def transfer(self):
    
    #generate command packet
    ES = 1
    EM1 = 1	
    EM2	= 0
    EPRE = 0
    EI1OC = 0
    EI2OC	= 0
    mode = (ES << 7) | (EM1 << 6) | (EM2<<5)|(EPRE<<4)|(EI1OC<<3)|(EI2OC<<2)
    timeout = 5
    rawRefPos0 = int(self.refPosition0*(1<<24))
    rawRefPos1 = int(self.refPosition1*(1<<24))
    rawRefVel0 = int(self.refVelocity0*(1<<11)*60.0/(2000*pi))
    rawRefVel1 = int(self.refVelocity1*(1<<11)*60.0/(2000*pi))
    rawRefIq0 = int(self.refCurrent0*(1<<10))
    rawRefIq1 = int(self.refCurrent1*(1<<10))
    rawIsat0 = int(self.iSatCurrent0*(1<<3))
    rawIsat1 = int(self.iSatCurrent1*(1<<3))
    rawKp0 = int(self.kp0*(1<<11)*(2*pi) )
    rawKp1 = int(self.kp1*(1<<11)*(2*pi) )
    rawKd0 = int(self.kd0*(1<<10)*(2*pi*1000./60.0) )
    rawKd1 = int(self.kd1*(1<<10)*(2*pi*1000./60.0) )
    commandPacket = bytearray(struct.pack(">BBiihhhhHHHHBBHI",mode,timeout,rawRefPos0,rawRefPos1,rawRefVel0,rawRefVel1,rawRefIq0,rawRefIq1,rawKp0,rawKp1,rawKd0,rawKd1, rawIsat0, rawIsat1, 0,0))
    crc=crc32(commandPacket[:-4])
    commandPacket[-4]=(crc>>24)&0xff
    commandPacket[-3]=(crc>>16)&0xff
    commandPacket[-2]=(crc>>8)&0xff
    commandPacket[-1]=(crc)&0xff
    GPIO.output(25,0) #enable CS
    sensorPacket = bytearray(self.spi.xfer(commandPacket))
    GPIO.output(25,1) #disable CS
    #print(commandPacket.hex(),checkcrc(commandPacket))
    #print(sensorPacket.hex(),checkcrc(sensorPacket))
    if checkcrc(sensorPacket):
    #decode received sensor packet
      data = struct.unpack(">H H i i h h h h xxxxxxxxxxxxxx",sensorPacket)
      self.is_system_enabled = data[0]&0b1000000000000000 != 0
      self.is_enabled0       = data[0]&0b0100000000000000 != 0
      self.is_ready0         = data[0]&0b0010000000000000 != 0
      self.is_enabled1       = data[0]&0b0001000000000000 != 0
      self.is_ready1         = data[0]&0b0000100000000000 != 0
      self.error             = data[0]&0b0000000000001111
      self.position0 = data[2] / (1<<24) * 2.0 * pi
      self.position1 = data[3] / (1<<24) * 2.0 * pi
      self.velocity0 = data[4] / (1<<11) * 2000*pi/60.0
      self.velocity1 = data[5] / (1<<11) * 2000*pi/60.0
      self.current0 = data[6]  / (1<<10)
      self.current1 = data[7]  / (1<<10)
      #print("velocity =", self.velocity0)
      #print("cur =", self.current0)
    else:
        print("Error: sensor frame is corrupted")
    if (self.error!=0):
        raise(Exception(f"Error from motor driver: Error {self.error}"))
class PID:
    def __init__(self,Kp,Ki,Kd, sat = 2.0, dt=0.001):
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.sat = sat
        self.u = 0.0
        self.ierr = 0.0
        self.dt = dt
    def compute(self, p, v, p_ref=0.0, v_ref=0.0):
        perr = p_ref-p
        verr = v_ref-v
        self.ierr = self.ierr + perr * self.dt
        if (self.ierr > self.sat) : 
            self.ierr = self.sat
        if (self.ierr < -self.sat) : 
            self.ierr = -self.sat
        self.u = self.Kp * perr + self.Kd * verr + self.Ki * self.ierr
        if (self.u > self.sat) : 
            self.u = self.sat
        if (self.u < -self.sat) : 
            self.u = -self.sat
        return self.u


