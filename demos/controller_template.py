#!/usr/bin/env python3
"""
Student controller template for odri_spi_rpi.

Structure:
1) Imports and user parameters
2) Driver initialization with offsets
3) Real-time control loop
4) Safe shutdown
"""

import time

from odri_spi_rpi import SPIuDriver

# ---------------------------------------------------------------------------
# User parameters
# ---------------------------------------------------------------------------
DT = 0.001
PRINT_EVERY = 200

# Replace these with your calibrated offsets from calibrate_offsets.py
OFFSET0 = 0.0
OFFSET1 = 0.0

# Move to a known start state before starting the control loop.
START_P0 = 0.0
START_P1 = 0.0


def main():
    # Initialize driver with absolute encoder mode and offsets.
    driver = SPIuDriver(absolutePositionMode=True, offsets=[OFFSET0, OFFSET1])

    print(f"Moving to start state: p0={START_P0}, p1={START_P1}")
    driver.goto(START_P0, START_P1)

    step = 0
    next_t = time.perf_counter()

    try:
        while True:
            # ----------------------------------------------------------------
            # 1) Command update (template: zero-current control)
            # ----------------------------------------------------------------
            driver.refCurrent0 = 0.0
            driver.refCurrent1 = 0.0

            # ----------------------------------------------------------------
            # 2) Exchange one frame with the motor driver
            #    - sends command values above
            #    - updates sensor values below
            # ----------------------------------------------------------------
            driver.transfer()

            # ----------------------------------------------------------------
            # 3) Read sensors (example)
            # ----------------------------------------------------------------
            position0 = driver.position0
            position1 = driver.position1
            velocity0 = driver.velocity0
            velocity1 = driver.velocity1

            # Example debug print (reduce frequency to keep loop light).
            if step % PRINT_EVERY == 0:
                print(
                    f"p0={position0:.3f} p1={position1:.3f} "
                    f"v0={velocity0:.3f} v1={velocity1:.3f}"
                )

            # TODO: add your custom controller logic here.

            step += 1
            next_t += DT
            while time.perf_counter() < next_t:
                pass
    except KeyboardInterrupt:
        print("Controller interrupted by user.")
    finally:
        driver.stop()
        driver.close()


if __name__ == "__main__":
    main()
