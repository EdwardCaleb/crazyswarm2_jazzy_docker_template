#!/usr/bin/env python3

import math
import signal

import rclpy

from crazyflie_py import Crazyswarm


# ============================================================
# Flight configuration
# ============================================================

TAKEOFF_HEIGHT = 0.40
TAKEOFF_DURATION = 2.5

LAND_HEIGHT = 0.04
LAND_DURATION = 2.5


# ============================================================
# Waypoints
#
# Format:
#
# ([x, y, z], yaw, travel_time, hold_time)
#
# Coordinates are ABSOLUTE because relative=False is used.
# ============================================================

CF1_WAYPOINTS = [
    ([0.20, 0.40, 0.40], math.radians(0),   2.0, 1.0),
    ([0.50, 0.40, 0.40], math.radians(0),   2.0, 1.0),
    ([0.50, 0.70, 0.40], math.radians(90),  2.0, 1.0),
    ([0.20, 0.70, 0.40], math.radians(180), 2.0, 1.0),
]


CF2_WAYPOINTS = [
    ([0.20, -0.20, 0.40], math.radians(0),   2.0, 1.0),
    ([0.50, -0.20, 0.40], math.radians(0),   2.0, 1.0),
    ([0.50,  0.10, 0.40], math.radians(90),  2.0, 1.0),
    ([0.20,  0.10, 0.40], math.radians(180), 2.0, 1.0),
]


# ============================================================
# Emergency state
# ============================================================

abort_requested = False
emergency_sent = False


def sigint_handler(signum, frame):
    """
    Ctrl+C handler.

    Do NOT call ROS services directly from the signal handler.
    We only set a flag. The main loop sends the emergency command.
    """

    global abort_requested

    if not abort_requested:
        print("\nCTRL+C detected -> EMERGENCY STOP requested")

    abort_requested = True


def emergency_stop(allcfs, time_helper):
    """
    Immediately emergency-stop every enabled Crazyflie.

    Motors are cut and the Crazyflies remain locked until rebooted.
    """

    global emergency_sent

    if emergency_sent:
        return

    emergency_sent = True

    print("\n========================================")
    print("!!! EMERGENCY STOP - ALL DRONES !!!")
    print("========================================")

    # Broadcast ROS service /all/emergency
    allcfs.emergency()

    # emergency() is asynchronous.
    # Keep ROS alive briefly so the request can be transmitted.
    end_time = time_helper.time() + 0.5

    while rclpy.ok() and time_helper.time() < end_time:
        rclpy.spin_once(
            time_helper.node,
            timeout_sec=0.01
        )


def wait_interruptible(duration, allcfs, time_helper):
    """
    Wait for 'duration' seconds while monitoring Ctrl+C.

    Returns:
        True  -> completed normally
        False -> emergency was requested
    """

    end_time = time_helper.time() + duration

    while time_helper.time() < end_time:

        if abort_requested:
            emergency_stop(allcfs, time_helper)
            return False

        rclpy.spin_once(
            time_helper.node,
            timeout_sec=0.01
        )

    return True


def main():

    # --------------------------------------------------------
    # Connect to the already-running Crazyswarm2 ROS server
    # --------------------------------------------------------

    swarm = Crazyswarm()

    time_helper = swarm.timeHelper
    allcfs = swarm.allcfs


    # --------------------------------------------------------
    # Install our Ctrl+C handler AFTER Crazyswarm/rclpy init
    # --------------------------------------------------------

    signal.signal(
        signal.SIGINT,
        sigint_handler
    )


    # --------------------------------------------------------
    # Get drones explicitly by name
    #
    # Do not depend on crazyflies[0], crazyflies[1] ordering.
    # --------------------------------------------------------

    try:
        cf1 = allcfs.crazyfliesByName["cf_1"]
        cf2 = allcfs.crazyfliesByName["cf_2"]

    except KeyError as exc:
        raise RuntimeError(
            f"Crazyflie not found: {exc}. "
            "Make sure cf_1 and cf_2 are enabled."
        )


    print("Crazyflies found:")
    print(f"  cf_1 -> {cf1.uri}")
    print(f"  cf_2 -> {cf2.uri}")

    print()
    print("Ctrl+C = IMMEDIATE EMERGENCY MOTOR STOP")
    print()


    # Both lists represent synchronized stages.
    if len(CF1_WAYPOINTS) != len(CF2_WAYPOINTS):
        raise RuntimeError(
            "CF1_WAYPOINTS and CF2_WAYPOINTS "
            "must contain the same number of stages."
        )


    flight_active = False

    try:

        # ====================================================
        # Arm
        # ====================================================

        print("Arming both drones...")

        allcfs.arm(True)

        flight_active = True

        if not wait_interruptible(
            1.0,
            allcfs,
            time_helper
        ):
            return


        # ====================================================
        # Takeoff
        #
        # Broadcast gives better synchronization than sending
        # two separate takeoff service calls.
        # ====================================================

        print(
            f"Taking off to {TAKEOFF_HEIGHT:.2f} m..."
        )

        allcfs.takeoff(
            targetHeight=TAKEOFF_HEIGHT,
            duration=TAKEOFF_DURATION
        )

        if not wait_interruptible(
            TAKEOFF_DURATION + 1.0,
            allcfs,
            time_helper
        ):
            return


        # ====================================================
        # Waypoint stages
        # ====================================================

        for stage, (wp1, wp2) in enumerate(
            zip(CF1_WAYPOINTS, CF2_WAYPOINTS),
            start=1
        ):

            # ----------------------------------------------
            # Unpack cf_1 waypoint
            # ----------------------------------------------

            pos1, yaw1, travel1, hold1 = wp1

            # ----------------------------------------------
            # Unpack cf_2 waypoint
            # ----------------------------------------------

            pos2, yaw2, travel2, hold2 = wp2


            print()
            print(f"Stage {stage}")

            print(
                f"  cf_1 -> {pos1}, "
                f"yaw={math.degrees(yaw1):.1f} deg, "
                f"time={travel1:.1f}s"
            )

            print(
                f"  cf_2 -> {pos2}, "
                f"yaw={math.degrees(yaw2):.1f} deg, "
                f"time={travel2:.1f}s"
            )


            # ----------------------------------------------
            # Check emergency before issuing new commands
            # ----------------------------------------------

            if abort_requested:
                emergency_stop(
                    allcfs,
                    time_helper
                )
                return


            # ----------------------------------------------
            # Send both trajectories
            #
            # goTo() is asynchronous, so these two calls are
            # issued almost immediately one after another.
            # ----------------------------------------------

            cf1.goTo(
                goal=pos1,
                yaw=yaw1,
                duration=travel1,
                relative=False
            )

            cf2.goTo(
                goal=pos2,
                yaw=yaw2,
                duration=travel2,
                relative=False
            )


            # ----------------------------------------------
            # Wait until BOTH trajectories should be done
            # ----------------------------------------------

            travel_time = max(
                travel1,
                travel2
            )

            if not wait_interruptible(
                travel_time,
                allcfs,
                time_helper
            ):
                return


            # ----------------------------------------------
            # Hover
            # ----------------------------------------------

            hold_time = max(
                hold1,
                hold2
            )

            if not wait_interruptible(
                hold_time,
                allcfs,
                time_helper
            ):
                return


        # ====================================================
        # Normal landing
        # ====================================================

        print()
        print("Landing both drones...")

        allcfs.land(
            targetHeight=LAND_HEIGHT,
            duration=LAND_DURATION
        )

        if not wait_interruptible(
            LAND_DURATION + 1.0,
            allcfs,
            time_helper
        ):
            return


        # ====================================================
        # Disarm
        # ====================================================

        print("Disarming both drones...")

        allcfs.arm(False)

        flight_active = False

        wait_interruptible(
            0.5,
            allcfs,
            time_helper
        )

        print()
        print("Flight complete.")


    except Exception as exc:

        print()
        print("Unexpected Python error:")
        print(exc)

        # If something unexpected happens while armed,
        # fail toward motor shutdown.
        if flight_active:
            emergency_stop(
                allcfs,
                time_helper
            )

        raise


if __name__ == "__main__":
    main()