#!/usr/bin/env python3
# Thomas Flayols - feb 2022
import time
from math import cos, pi, sin

from odri_spi_rpi import SPIuDriver

DT = 0.001
AMPLITUDE_RAD = 2.0 * pi  # 1 turn amplitude
FREQUENCY_HZ = 0.2
OMEGA = 2.0 * pi * FREQUENCY_HZ
KP = 1.0
KD = 0.05


def main():
    driver = SPIuDriver(absolutePositionMode=True)

    # Onboard PD (computed on motor driver side).
    driver.kp0 = KP
    driver.kp1 = KP
    driver.kd0 = KD
    driver.kd1 = KD
    driver.refCurrent0 = 0.0
    driver.refCurrent1 = 0.0

    try:
        driver.transfer()
        driver.goto(0, 0)
        start_t = time.perf_counter()
        next_t = start_t
        loop_id = 0

        while True:
            elapsed = time.perf_counter() - start_t
            target_position = AMPLITUDE_RAD * sin(OMEGA * elapsed)
            target_velocity = AMPLITUDE_RAD * OMEGA * cos(OMEGA * elapsed)

            driver.refPosition0 = target_position
            driver.refPosition1 = target_position
            driver.refVelocity0 = target_velocity
            driver.refVelocity1 = target_velocity
            driver.transfer()

            if loop_id % 100 == 0:
                print(
                    f"target={target_position:.3f} "
                    f"p0={driver.position0:.3f} p1={driver.position1:.3f} "
                    f"i0={driver.current0:.3f} i1={driver.current1:.3f}"
                )
            loop_id += 1

            next_t += DT
            while time.perf_counter() < next_t:
                pass
    finally:
        driver.stop()
        driver.close()


if __name__ == "__main__":
    main()
