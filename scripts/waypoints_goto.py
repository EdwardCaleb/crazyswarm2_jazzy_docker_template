#!/usr/bin/env python3

import math
import signal

import rclpy

from crazyflie_py import Crazyswarm


# ------------------------------------------------------------
# Flight configuration
# ------------------------------------------------------------

TAKEOFF_HEIGHT = 0.40
TAKEOFF_DURATION = 2.5

LAND_HEIGHT = 0.04
LAND_DURATION = 2.5

# True  -> waypoints are offsets
# False -> waypoints are absolute coordinates
RELATIVE = False


# ------------------------------------------------------------
# Waypoints
#
# Keep your own coordinates here.
# Format:
#
# ([x, y, z], yaw, travel_time, hold_time)
# ------------------------------------------------------------

WAYPOINTS = [
    ([0.20, 0.20, 0.40], math.radians(0),   2.0, 1.0),
    ([0.50, 0.20, 0.40], math.radians(90),  2.0, 1.0),
    ([0.50, 0.50, 0.40], math.radians(180), 2.0, 1.0),
    ([0.20, 0.50, 0.40], math.radians(270), 2.0, 1.0),
]



# ------------------------------------------------------------
# Emergency state
# ------------------------------------------------------------

abort_requested = False


def sigint_handler(signum, frame):
    """
    Called when Ctrl+C is pressed.

    Important:
    Do not perform ROS operations directly inside the signal handler.
    Only set a flag. The main thread will send the emergency command.
    """
    global abort_requested

    if not abort_requested:
        print("\nCTRL+C detected: EMERGENCY STOP requested")

    abort_requested = True


def sleep_interruptible(time_helper, duration):
    """
    Sleep while continuing to process ROS callbacks.

    Returns:
        True  -> duration completed normally
        False -> Ctrl+C was pressed
    """

    end_time = time_helper.time() + duration

    while time_helper.time() < end_time:

        if abort_requested:
            return False

        rclpy.spin_once(
            time_helper.node,
            timeout_sec=0.01
        )

    return True


def emergency_stop(cf, time_helper):
    """
    Send an immediate emergency stop.

    This cuts motor power and leaves the Crazyflie locked.
    A reboot/power-cycle is required afterwards.
    """

    print("!!! EMERGENCY STOP !!!")

    cf.emergency()

    # Give ROS a short opportunity to transmit the async service request.
    end_time = time_helper.time() + 0.3

    while rclpy.ok() and time_helper.time() < end_time:
        rclpy.spin_once(
            time_helper.node,
            timeout_sec=0.01
        )


def main():

    global abort_requested

    swarm = Crazyswarm()

    time_helper = swarm.timeHelper

    cf = swarm.allcfs.crazyflies[0]

    # Crazyswarm() has already called rclpy.init().
    # Override SIGINT afterwards so Ctrl+C does not immediately
    # tear down ROS before we send the emergency command.
    signal.signal(
        signal.SIGINT,
        sigint_handler
    )

    print(f"Using Crazyflie: {cf.prefix}")
    print("Ctrl+C = IMMEDIATE EMERGENCY MOTOR STOP")

    armed = False

    # --------------------------------------------------------
    # Arm
    # --------------------------------------------------------

    print("Arming...")
    cf.arm(True)
    armed = True

    if not sleep_interruptible(time_helper, 1.0):
        emergency_stop(cf, time_helper)
        return

    # --------------------------------------------------------
    # Takeoff
    # --------------------------------------------------------

    print(f"Taking off to {TAKEOFF_HEIGHT} m")

    cf.takeoff(
        targetHeight=TAKEOFF_HEIGHT,
        duration=TAKEOFF_DURATION
    )

    if not sleep_interruptible(
        time_helper,
        TAKEOFF_DURATION + 1.0
    ):
        emergency_stop(cf, time_helper)
        return

    # --------------------------------------------------------
    # Waypoints
    # --------------------------------------------------------

    for i, (position, yaw, travel_time, hold_time) in enumerate(
        WAYPOINTS,
        start=1
    ):

        print(
            f"Waypoint {i}: "
            f"position={position}, "
            f"yaw={math.degrees(yaw):.1f} deg, "
            f"travel={travel_time}s, "
            f"hold={hold_time}s"
        )

        cf.goTo(
            goal=position,
            yaw=yaw,
            duration=travel_time,
            relative=RELATIVE
        )

        if not sleep_interruptible(
            time_helper,
            travel_time
        ):
            emergency_stop(cf, time_helper)
            return

        if not sleep_interruptible(
            time_helper,
            hold_time
        ):
            emergency_stop(cf, time_helper)
            return

    # --------------------------------------------------------
    # Normal landing
    # --------------------------------------------------------

    print("Landing...")

    cf.land(
        targetHeight=LAND_HEIGHT,
        duration=LAND_DURATION
    )

    if not sleep_interruptible(
        time_helper,
        LAND_DURATION + 1.0
    ):
        emergency_stop(cf, time_helper)
        return

    # --------------------------------------------------------
    # Disarm
    # --------------------------------------------------------

    print("Disarming...")

    cf.arm(False)
    armed = False

    sleep_interruptible(time_helper, 0.5)

    print("Flight complete.")


if __name__ == "__main__":
    main()