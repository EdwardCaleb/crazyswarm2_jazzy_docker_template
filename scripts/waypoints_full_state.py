#!/usr/bin/env python3

import signal

import numpy as np
import rclpy
import transforms3d

from crazyflie_py import Crazyswarm


# ============================================================
# Configuration
# ============================================================

DRONE_NAME = "cf_1"

RATE = 30.0

TAKEOFF_HEIGHT = 0.40
TAKEOFF_DURATION = 2.5

LAND_HEIGHT = 0.04
LAND_DURATION = 2.5


# Absolute coordinates:
#
# ([x, y, z], travel_time, hold_time)
#
# Change these coordinates to fit your OptiTrack flight area.
# WAYPOINTS = [
#     ([0.20, 0.50, 0.40], 2.0, 1.0),
#     ([0.50, 0.50, 0.40], 2.0, 1.0),
#     ([0.50, 0.80, 0.40], 2.0, 1.0),
#     ([0.20, 0.80, 0.40], 2.0, 1.0),
# ]


WAYPOINTS = [
    ([0.20, 0.50, 0.40], 2.0, 1.0),
    ([0.50, 0.50, 0.40], 2.0, 1.0),
    ([0.50, 0.80, 0.40], 2.0, 1.0),
    ([0.20, 0.80, 0.40], 2.0, 1.0),
]




# ============================================================
# Emergency handling
# ============================================================

abort_requested = False
emergency_sent = False


def sigint_handler(signum, frame):
    global abort_requested

    if not abort_requested:
        print("\nCTRL+C -> EMERGENCY STOP requested")

    abort_requested = True


def emergency_stop(cf, time_helper):
    global emergency_sent

    if emergency_sent:
        return

    emergency_sent = True

    print()
    print("================================")
    print("!!! EMERGENCY STOP !!!")
    print("================================")

    cf.emergency()

    # emergency() is asynchronous.
    # Keep ROS alive briefly so the service request can be sent.
    end_time = time_helper.time() + 0.5

    while rclpy.ok() and time_helper.time() < end_time:
        rclpy.spin_once(
            time_helper.node,
            timeout_sec=0.01
        )


# ============================================================
# 7th-order smooth interpolation
# ============================================================

def smooth_segment(p0, p1, t, duration):
    """
    Evaluate a smooth trajectory from p0 to p1.

    Boundary conditions at both ends:

        velocity     = 0
        acceleration = 0
        jerk         = 0

    Returns:
        position
        velocity
        acceleration
    """

    u = np.clip(t / duration, 0.0, 1.0)

    # Position scaling
    s = (
        35.0 * u**4
        - 84.0 * u**5
        + 70.0 * u**6
        - 20.0 * u**7
    )

    # ds / du
    ds = (
        140.0 * u**3
        - 420.0 * u**4
        + 420.0 * u**5
        - 140.0 * u**6
    )

    # d²s / du²
    dds = (
        420.0 * u**2
        - 1680.0 * u**3
        + 2100.0 * u**4
        - 840.0 * u**5
    )

    delta = p1 - p0

    position = p0 + delta * s

    velocity = delta * ds / duration

    acceleration = delta * dds / (duration**2)

    return position, velocity, acceleration


# ============================================================
# Stream one trajectory segment
# ============================================================

def execute_segment(
    cf,
    time_helper,
    start_position,
    target_position,
    duration,
    yaw,
):

    start_time = time_helper.time()

    omega = np.zeros(3)

    while True:

        if abort_requested:
            emergency_stop(cf, time_helper)
            return False

        t = time_helper.time() - start_time

        if t >= duration:
            break

        position, velocity, acceleration = smooth_segment(
            start_position,
            target_position,
            t,
            duration
        )

        cf.cmdFullState(
            position,
            velocity,
            acceleration,
            yaw,
            omega
        )

        time_helper.sleepForRate(RATE)

    # Send exact final state.
    cf.cmdFullState(
        target_position,
        np.zeros(3),
        np.zeros(3),
        yaw,
        np.zeros(3)
    )

    return True


# ============================================================
# Hold a waypoint
# ============================================================

def hold_position(
    cf,
    time_helper,
    position,
    duration,
    yaw,
):
    """
    Important:

    We cannot simply sleep after entering low-level mode.

    cmdFullState is a STREAMING interface, so during a hover we
    continue sending the same position with zero velocity and
    acceleration.
    """

    start_time = time_helper.time()

    while time_helper.time() - start_time < duration:

        if abort_requested:
            emergency_stop(cf, time_helper)
            return False

        cf.cmdFullState(
            position,
            np.zeros(3),
            np.zeros(3),
            yaw,
            np.zeros(3)
        )

        time_helper.sleepForRate(RATE)

    return True


# ============================================================
# Main
# ============================================================

def main():

    swarm = Crazyswarm()

    allcfs = swarm.allcfs
    time_helper = swarm.timeHelper

    signal.signal(
        signal.SIGINT,
        sigint_handler
    )


    # --------------------------------------------------------
    # Select drone explicitly
    # --------------------------------------------------------

    if DRONE_NAME not in allcfs.crazyfliesByName:
        raise RuntimeError(
            f"{DRONE_NAME} is not enabled in crazyflies.yaml"
        )

    cf = allcfs.crazyfliesByName[DRONE_NAME]


    print(f"Using {DRONE_NAME}")
    print(f"URI: {cf.uri}")
    print(f"cmdFullState rate: {RATE:.1f} Hz")
    print("Ctrl+C = EMERGENCY STOP")
    print()


    flight_active = False


    try:

        # ====================================================
        # Arm
        # ====================================================

        print("Arming...")

        cf.arm(True)

        flight_active = True

        time_helper.sleep(1.0)


        # ====================================================
        # High-level takeoff
        # ====================================================

        print(
            f"Taking off to {TAKEOFF_HEIGHT:.2f} m..."
        )

        cf.takeoff(
            targetHeight=TAKEOFF_HEIGHT,
            duration=TAKEOFF_DURATION
        )

        time_helper.sleep(
            TAKEOFF_DURATION + 1.0
        )


        if abort_requested:
            emergency_stop(cf, time_helper)
            return


        # ====================================================
        # Get actual starting position
        #
        # /cf_1/pose is continuously updating cf.position.
        # ====================================================

        current_position = np.array(
            cf.get_position(),
            dtype=float
        )


        # Get current yaw so this first experiment changes only
        # XYZ position, not heading.
        pose = cf.get_pose()

        if not pose:
            raise RuntimeError(
                "No pose received from the Crazyflie."
            )


        q = pose["orientation"]

        roll, pitch, yaw = transforms3d.euler.quat2euler(
            [
                q.w,
                q.x,
                q.y,
                q.z
            ]
        )


        print()
        print(
            "Starting full-state position:",
            current_position
        )

        print(
            f"Holding yaw at {np.degrees(yaw):.1f} deg"
        )


        # ====================================================
        # cmdFullState waypoint sequence
        # ====================================================

        for index, waypoint in enumerate(
            WAYPOINTS,
            start=1
        ):

            target, travel_time, hold_time = waypoint

            target = np.array(
                target,
                dtype=float
            )


            print()
            print(
                f"Waypoint {index}/{len(WAYPOINTS)}"
            )

            print(
                f"  from: {current_position}"
            )

            print(
                f"  to:   {target}"
            )

            print(
                f"  travel: {travel_time:.2f} s"
            )

            print(
                f"  hold:   {hold_time:.2f} s"
            )


            # ----------------------------------------------
            # Smooth trajectory to waypoint
            # ----------------------------------------------

            success = execute_segment(
                cf,
                time_helper,
                current_position,
                target,
                travel_time,
                yaw
            )

            if not success:
                return


            # ----------------------------------------------
            # Hold the waypoint using streaming setpoints
            # ----------------------------------------------

            success = hold_position(
                cf,
                time_helper,
                target,
                hold_time,
                yaw
            )

            if not success:
                return


            current_position = target


        # ====================================================
        # Return from low-level to high-level mode
        # ====================================================

        print()
        print("Full-state trajectory complete.")

        print(
            "Releasing low-level streaming control..."
        )

        cf.notifySetpointsStop()


        # ====================================================
        # High-level landing
        #
        # This is the same transition used by the official
        # Crazyswarm2 cmd_full_state example.
        # ====================================================

        print("Landing...")

        cf.land(
            targetHeight=LAND_HEIGHT,
            duration=LAND_DURATION
        )

        time_helper.sleep(
            LAND_DURATION + 1.0
        )


        # ====================================================
        # Disarm
        # ====================================================

        print("Disarming...")

        cf.arm(False)

        flight_active = False

        time_helper.sleep(0.5)


        print()
        print("Flight complete.")


    except Exception as exc:

        print()
        print("Unexpected error:")
        print(exc)


        if flight_active:
            emergency_stop(
                cf,
                time_helper
            )

        raise


if __name__ == "__main__":
    main()
