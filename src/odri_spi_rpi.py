#!/usr/bin/env python3

#Thomas Flayols - feb 2022
#https://github.com/thomasfla/odri-spi-rpi
#https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_%C2%B5Driver_SPI_interface.md
import time
import struct
from math import pi

import RPi.GPIO as GPIO
import spidev


CS_PIN = 25
ERROR_DESCRIPTIONS = {
  0: "No error",
  1: "Encoder_1 error too high",
  2: "Timeout for receiving cmd exceeded",
  3: "Motor temperature reached critical value",
  4: "Unused error",
  5: "Position roll-over occurred",
  6: "Encoder_2 error",
  7: "Motor DRV nFault error",
}


def crc32(buf):
  crc=0xffffffff
  for val in buf:
    crc ^= val << 24
    for _ in range(8):
      crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
  return crc

def checkcrc(buf):
  crc = crc32(buf[:-4])
  # Sensor frame returns CRC with low/high 16-bit words swapped vs command frame.
  low_word = buf[-4] * 256 + buf[-3]
  high_word = buf[-2] * 256 + buf[-1]
  return (crc & 0xffff == low_word and (crc & 0xFFFF0000) >> 16 == high_word)


def get_error_description(error_code):
  return ERROR_DESCRIPTIONS.get(error_code, f"Unknown error code {error_code}")


def build_trapezoidal_profile(distance, max_velocity, max_acceleration):
  if max_velocity <= 0.0:
    raise ValueError("max_velocity must be > 0")
  if max_acceleration <= 0.0:
    raise ValueError("max_acceleration must be > 0")

  sign = 1.0 if distance >= 0.0 else -1.0
  distance_abs = abs(distance)
  if distance_abs == 0.0:
    return {
      "sign": sign,
      "distance": 0.0,
      "t_acc": 0.0,
      "t_cruise": 0.0,
      "t_total": 0.0,
      "v_peak": 0.0,
    }

  t_acc = max_velocity / max_acceleration
  d_acc = 0.5 * max_acceleration * t_acc * t_acc

  # Triangular profile if we cannot reach max_velocity.
  if 2.0 * d_acc >= distance_abs:
    t_acc = (distance_abs / max_acceleration) ** 0.5
    t_cruise = 0.0
    v_peak = max_acceleration * t_acc
  else:
    t_cruise = (distance_abs - 2.0 * d_acc) / max_velocity
    v_peak = max_velocity

  return {
    "sign": sign,
    "distance": distance_abs,
    "t_acc": t_acc,
    "t_cruise": t_cruise,
    "t_total": 2.0 * t_acc + t_cruise,
    "v_peak": v_peak,
  }


def sample_trapezoidal_profile(profile, t, max_acceleration):
  if t <= 0.0 or profile["distance"] == 0.0:
    return 0.0, 0.0

  t_acc = profile["t_acc"]
  t_cruise = profile["t_cruise"]
  t_total = profile["t_total"]
  v_peak = profile["v_peak"]
  d_acc = 0.5 * max_acceleration * t_acc * t_acc

  if t < t_acc:
    pos = 0.5 * max_acceleration * t * t
    vel = max_acceleration * t
  elif t < (t_acc + t_cruise):
    pos = d_acc + v_peak * (t - t_acc)
    vel = v_peak
  elif t < t_total:
    td = t - t_acc - t_cruise
    pos = d_acc + v_peak * t_cruise + v_peak * td - 0.5 * max_acceleration * td * td
    vel = v_peak - max_acceleration * td
  else:
    pos = profile["distance"]
    vel = 0.0

  return profile["sign"] * pos, profile["sign"] * vel

class SPIuDriver:
  def __init__(self, waitForInit = True, absolutePositionMode = False, offsets=None):

    if offsets is None:
      offsets = (0.0, 0.0)
    if len(offsets) != 2:
      raise ValueError("offsets must contain two values: [offset0, offset1]")
    self._closed = False

    #Configure CS pin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(CS_PIN,GPIO.OUT)
    GPIO.output(CS_PIN,0)

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

    self.offset0 = offsets[0]
    self.offset1 = offsets[1]

    self.EI1OC = 1 if absolutePositionMode else 0
    self.EI2OC = 1 if absolutePositionMode else 0
    self.refPosition0 = 0
    self.refPosition1= 0
    self.refVelocity0= 0
    self.refVelocity1= 0
    self.refCurrent0= 0
    self.refCurrent1= 0
    self.iSatCurrent0= 5.0
    self.iSatCurrent1= 5.0
    self.kp0= 0
    self.kp1= 0
    self.kd0= 0
    self.kd1= 0
    self.timeout = 20

    self.error = -1
    #wait for system enable
    if waitForInit:
      print(">> Calibrating motors, please wait")
      while(not self.is_ready0 or not self.is_ready1):
        self.transfer()
        time.sleep(0.001)
    if absolutePositionMode:
      if (not self.has_index_been_detected0 or not self.has_index_been_detected1):
          print(">> Waiting for index pulse to have absolute position reference, please move the motors manually")
          displayedIndex0 = False
          displayedIndex1 = False
          while(not self.has_index_been_detected0 or not self.has_index_been_detected1):
            self.transfer()
            if self.has_index_been_detected0 == True and displayedIndex0 == False:
                print (" >> Index 0 detected !")
                displayedIndex0 = True
            if self.has_index_been_detected1 == True and displayedIndex1 == False:
                print (" >> Index 1 detected !")
                displayedIndex1 = True
            time.sleep(0.001)
    print ("ready!")
  def transfer(self):

    #generate command packet
    ES = 1
    EM1 = 1	
    EM2	= 1
    EPRE = 1
    EI1OC = self.EI1OC
    EI2OC	= self.EI2OC
    mode = (ES << 7) | (EM1 << 6) | (EM2<<5)|(EPRE<<4)|(EI1OC<<3)|(EI2OC<<2)
    timeout = self.timeout
    rawRefPos0 = int((self.refPosition0-self.offset0)/(2.*pi)*(1<<24) )
    rawRefPos1 = int((self.refPosition1-self.offset1)/(2.*pi)*(1<<24))
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
    GPIO.output(CS_PIN,0) #enable CS
    sensorPacket = bytearray(self.spi.xfer(commandPacket))
    GPIO.output(CS_PIN,1) #disable CS
    #print(commandPacket.hex(),checkcrc(commandPacket))
    #print(sensorPacket.hex(),checkcrc(sensorPacket))
    if checkcrc(sensorPacket):
    #decode received sensor packet
      data = struct.unpack(">H H i i h h h h xxxxxxxxxxxxxx",sensorPacket)
      self.is_system_enabled        = data[0]&0b1000000000000000 != 0
      self.is_enabled0              = data[0]&0b0100000000000000 != 0
      self.is_ready0                = data[0]&0b0010000000000000 != 0
      self.is_enabled1              = data[0]&0b0001000000000000 != 0
      self.is_ready1                = data[0]&0b0000100000000000 != 0
      self.has_index_been_detected0 = data[0]&0b0000010000000000 != 0
      self.has_index_been_detected1 = data[0]&0b0000001000000000 != 0

      self.error = data[0]&0b0000000000001111
      self.error_code = self.error
      self.position0 = data[2] / (1<<24) * 2.0 * pi + self.offset0
      self.position1 = data[3] / (1<<24) * 2.0 * pi + self.offset1
      self.velocity0 = data[4] / (1<<11) * 2000*pi/60.0
      self.velocity1 = data[5] / (1<<11) * 2000*pi/60.0
      self.current0 = data[6]  / (1<<10)
      self.current1 = data[7]  / (1<<10)
      #print("velocity =", self.velocity0)
      #print("cur =", self.current0)
    else:
        raise RuntimeError("Error: sensor frame is corrupted, is uDriver powered on?")
    if (self.error!=0):
        raise RuntimeError(f"Error from motor driver ({self.error}): {get_error_description(self.error)}")
    #print(sensorPacket.hex())
  def goto(self, p0, p1, max_velocity=6.0, max_acceleration=20.0, kp=3.0, kd=0.06, dt=0.001):
        """Move both motors to targets using onboard PD and trapezoidal profiles."""
        start0 = self.position0
        start1 = self.position1
        distance0 = p0 - start0
        distance1 = p1 - start1

        profile0 = build_trapezoidal_profile(distance0, max_velocity, max_acceleration)
        profile1 = build_trapezoidal_profile(distance1, max_velocity, max_acceleration)
        motion_time = max(profile0["t_total"], profile1["t_total"])

        prev_kp0 = self.kp0
        prev_kp1 = self.kp1
        prev_kd0 = self.kd0
        prev_kd1 = self.kd1
        self.kp0 = kp
        self.kp1 = kp
        self.kd0 = kd
        self.kd1 = kd
        self.refCurrent0 = 0.0
        self.refCurrent1 = 0.0

        try:
            t0 = time.perf_counter()
            next_t = t0
            while True:
                elapsed = time.perf_counter() - t0
                rel_pos0, rel_vel0 = sample_trapezoidal_profile(profile0, elapsed, max_acceleration)
                rel_pos1, rel_vel1 = sample_trapezoidal_profile(profile1, elapsed, max_acceleration)
                self.refPosition0 = start0 + rel_pos0
                self.refPosition1 = start1 + rel_pos1
                self.refVelocity0 = rel_vel0
                self.refVelocity1 = rel_vel1
                self.refCurrent0 = 0.0
                self.refCurrent1 = 0.0
                self.transfer()

                if elapsed >= motion_time:
                    break

                next_t += dt
                while time.perf_counter() < next_t:
                    pass

            self.refPosition0 = p0
            self.refPosition1 = p1
            self.refVelocity0 = 0.0
            self.refVelocity1 = 0.0
            self.refCurrent0 = 0.0
            self.refCurrent1 = 0.0
            self.transfer()
        finally:
            self.kp0 = prev_kp0
            self.kp1 = prev_kp1
            self.kd0 = prev_kd0
            self.kd1 = prev_kd1
  def stop(self):
        self.EI1OC = 0
        self.EI2OC = 0
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
        self.timeout = 0
        dt=0.001
        t = time.perf_counter()
        for i in range(100):
            self.transfer() #transfer
            #wait for next control cycle
            t +=dt
            while(time.perf_counter()-t<dt):
                pass
  def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.spi.close()
        finally:
            GPIO.cleanup(CS_PIN)

  def __enter__(self):
        return self

  def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

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
