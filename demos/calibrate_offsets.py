#!/usr/bin/env python3
import time
from math import pi

from odri_spi_rpi import SPIuDriver

def wrap_to_pi(angle):
    return ((angle + pi) % (2.0 * pi)) - pi


def main():
    driver = SPIuDriver(absolutePositionMode=True, offsets=[0.0, 0.0])

    try:
        # Keep the controller fully passive for calibration.
        driver.kp0 = 0.0
        driver.kp1 = 0.0
        driver.kd0 = 0.0
        driver.kd1 = 0.0
        driver.refPosition0 = 0.0
        driver.refPosition1 = 0.0
        driver.refVelocity0 = 0.0
        driver.refVelocity1 = 0.0
        driver.refCurrent0 = 0.0
        driver.refCurrent1 = 0.0
        # Avoid timeout while waiting at the terminal prompt.
        driver.timeout = 0        
        driver.transfer()
        print("Move the mechanism manually to its mechanical zero position, then press Enter.")
        input()
        driver.transfer()

        offset0 = -wrap_to_pi(driver.position0)
        offset1 = -wrap_to_pi(driver.position1)
        print("")
        print("Calibration result:")
        print(f"offsets=[{offset0},{offset1}]")
        print("Use these values in your next SPIuDriver(..., offsets=[...]) call.")
    finally:
        driver.stop()
        driver.close()


if __name__ == "__main__":
    main()
