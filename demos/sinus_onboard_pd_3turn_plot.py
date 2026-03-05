#!/usr/bin/env python3
# Thomas Flayols - feb 2022
import time
from math import cos, pi, sin

import matplotlib.pyplot as plt

from odri_spi_rpi import SPIuDriver

DT = 0.001
AMPLITUDE_RAD = 2.0 * pi  # 1 turn amplitude
FREQUENCY_HZ = 0.2
OMEGA = 2.0 * pi * FREQUENCY_HZ
KP = 1.0
KD = 0.05
TARGET_TRAVEL_RAD = 3.0 * 2.0 * pi  # 3 turns


def plot_signals(times_s, positions_rad, velocities_rad_s):
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    axes[0].plot(times_s, positions_rad, label="position0")
    axes[0].set_ylabel("Position [rad]")
    axes[0].grid(True)
    axes[0].legend(loc="upper right")

    axes[1].plot(times_s, velocities_rad_s, label="velocity0")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Velocity [rad/s]")
    axes[1].grid(True)
    axes[1].legend(loc="upper right")

    fig.suptitle("3-Turn Sinus Run (sampled at dt)")
    fig.tight_layout()
    plt.show()


def main():
    driver = SPIuDriver(absolutePositionMode=True)

    # Onboard PD (computed on motor driver side).
    driver.kp0 = KP
    driver.kp1 = KP
    driver.kd0 = KD
    driver.kd1 = KD
    driver.refCurrent0 = 0.0
    driver.refCurrent1 = 0.0

    times = []
    positions = []
    velocities = []

    cumulative_travel = 0.0

    try:
        driver.transfer()
        driver.goto(0, 0)

        start_t = time.perf_counter()
        next_t = start_t
        prev_t = 0.0
        prev_pos = driver.position0
        prev_vel = driver.velocity0

        times.append(prev_t)
        positions.append(prev_pos)
        velocities.append(prev_vel)

        while True:
            elapsed = time.perf_counter() - start_t
            target_position = AMPLITUDE_RAD * sin(OMEGA * elapsed)
            target_velocity = AMPLITUDE_RAD * OMEGA * cos(OMEGA * elapsed)

            driver.refPosition0 = target_position
            driver.refPosition1 = target_position
            driver.refVelocity0 = target_velocity
            driver.refVelocity1 = target_velocity
            driver.transfer()

            cur_pos = driver.position0
            cur_vel = driver.velocity0
            delta_pos = cur_pos - prev_pos
            step_travel = abs(delta_pos)

            if step_travel > 0.0 and cumulative_travel + step_travel >= TARGET_TRAVEL_RAD:
                # Interpolate the last sample so the stop point is exactly at 3 turns.
                remaining = TARGET_TRAVEL_RAD - cumulative_travel
                alpha = remaining / step_travel
                final_t = prev_t + alpha * (elapsed - prev_t)
                final_pos = prev_pos + alpha * delta_pos
                final_vel = prev_vel + alpha * (cur_vel - prev_vel)

                times.append(final_t)
                positions.append(final_pos)
                velocities.append(final_vel)

                print("Reached exactly 3 turns of cumulative travel.")
                break

            cumulative_travel += step_travel
            times.append(elapsed)
            positions.append(cur_pos)
            velocities.append(cur_vel)

            prev_t = elapsed
            prev_pos = cur_pos
            prev_vel = cur_vel

            next_t += DT
            while time.perf_counter() < next_t:
                pass
    finally:
        driver.stop()
        driver.close()

    # Plot only after control has ended.
    plot_signals(times, positions, velocities)


if __name__ == "__main__":
    main()
