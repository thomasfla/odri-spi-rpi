#!/usr/bin/env python3

#Thomas Flayols - feb 2022
#https://github.com/thomasfla/odri-spi-rpi

import struct
import spidev

def init_spi():
  pass

def encode_command_packet():
  mode=0x00
  position_ref=0.0
  velocity_ref=0.0
  current_ref=0.0
  kp=0.0
  kd=0.0
  i_sat=0.0
  
  
def decode_sensor_packet():
  is_enabled =0
  error_code = 0
  adc0
  adc1 
  position0 = 0
  position1 = 0
  velocity0 = 0
  velocity1 = 0
  current0 = 0
  current1 = 0
  is_enabled0 = 0
  is_enabled1= 0
  is_ready0 = 0
  is_ready1 = 0
  has_index_been_detected0 =0
  has_index_been_detected1 =0
  index_toggle_bit0 = 0
  index_toggle_bit1 = 0


def print_packet(packet):
  '''Print raw packet, checking CRC, for debug'''
  pass
  
