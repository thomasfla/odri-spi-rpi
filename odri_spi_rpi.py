#!/usr/bin/env python3

#Thomas Flayols - feb 2022
#https://github.com/thomasfla/odri-spi-rpi
#https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_%C2%B5Driver_SPI_interface.md
import struct
import spidev
import RPi.GPIO as GPIO
import spidev


def checkcrc(buf):
    crc=0xffffffff
    for val in buf[:-4]:
        crc ^= val << 24
        for _ in range(8):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
    if (crc&0xffff == buf[-4]*256+buf[-3] and
       (crc&0xFFFF0000)>>16 == buf[-2]*256+buf[-1] ): #todo, clean
        return True
    return False

class SPIuDriver:
  def __init__(self):
    
    #Configure CS pin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(25,GPIO.OUT)
    GPIO.output(25,0)

    #Initialise SPI
    self.spi = spidev.SpiDev()
    self.spi.open(0, 0)
    self.spi.mode=0
    self.spi.max_speed_hz = 1000000
    
    # Allocate all variables
    self.is_enabled =0
    self.error_code = 0
    self.adc0 = 0
    self.adc1 = 0
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
    self.has_index_been_detected0 =0
    self.has_index_been_detected1 =0
    self.index_toggle_bit0 = 0
    self.index_toggle_bit1 = 0

  def transfer(self):
    GPIO.output(25,0) #enable CS
    res = self.spi.xfer([0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
    GPIO.output(25,1) #disable CS
    print(bytearray(res).hex(),checkcrc(res))

  def encode_command_packet():
    mode=0x00
    position_ref=0.0
    velocity_ref=0.0
    current_ref=0.0
    kp=0.0
    kd=0.0
    i_sat=0.0
  
  
