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
# Drones participating in this experiment
# ============================================================

DRONE_NAMES = [
    "cf_1",
    # "cf_2",
    # "cf_3",
    "cf_4",
    # "cf_7",
]


# ============================================================
# Absolute waypoints
#
# Format:
#
# ([x, y, z], yaw, travel_time, hold_time)
#
# Each drone must currently have the same number of stages.
# ============================================================

WAYPOINTS = {

    "cf_1": [
        ([0.20,  0.50, 0.40], math.radians(0), 2.0, 1.0),
        ([0.50,  0.50, 0.40], math.radians(0), 2.0, 1.0),
        ([0.50,  0.80, 0.40], math.radians(0), 2.0, 1.0),
    ],

    "cf_4": [
        ([0.20,  0.00, 0.40], math.radians(0), 2.0, 1.0),
        ([0.50,  0.00, 0.40], math.radians(0), 2.0, 1.0),
        ([0.50,  0.30, 0.40], math.radians(0), 2.0, 1.0),
    ],

    "cf_7": [
        ([0.20, -0.50, 0.40], math.radians(0), 2.0, 1.0),
        ([0.50, -0.50, 0.40], math.radians(0), 2.0, 1.0),
        ([0.50, -0.20, 0.40], math.radians(0), 2.0, 1.0),
    ],
}


# ============================================================
# Emergency state
# ============================================================

abort_requested = False
emergency_sent = False


def sigint_handler(signum, frame):

    global abort_requested

    if not abort_requested:
        print("\nCTRL+C -> EMERGENCY STOP requested")

    abort_requested = True


def emergency_stop(drones, time_helper):

    global emergency_sent

    if emergency_sent:
        return

    emergency_sent = True

    print()
    print("========================================")
    print("!!! EMERGENCY STOP - SELECTED DRONES !!!")
    print("========================================")

    for name, cf in drones.items():
        print(f"Emergency -> {name}")
        cf.emergency()

    # Give the asynchronous ROS service calls time to leave.
    end_time = time_helper.time() + 0.5

    while rclpy.ok() and time_helper.time() < end_time:
        rclpy.spin_once(
            time_helper.node,
            timeout_sec=0.01
        )


def wait_interruptible(duration, drones, time_helper):

    end_time = time_helper.time() + duration

    while time_helper.time() < end_time:

        if abort_requested:
            emergency_stop(
                drones,
                time_helper
            )
            return False

        rclpy.spin_once(
            time_helper.node,
            timeout_sec=0.01
        )

    return True


def main():

    swarm = Crazyswarm()

    allcfs = swarm.allcfs
    time_helper = swarm.timeHelper


    # --------------------------------------------------------
    # Install Ctrl+C handler
    # --------------------------------------------------------

    signal.signal(
        signal.SIGINT,
        sigint_handler
    )


    # --------------------------------------------------------
    # Find requested drones explicitly by name
    # --------------------------------------------------------

    drones = {}

    for name in DRONE_NAMES:

        if name not in allcfs.crazyfliesByName:
            raise RuntimeError(
                f"{name} was not found. "
                "Check crazyflies.yaml and make sure it is enabled."
            )

        drones[name] = allcfs.crazyfliesByName[name]


    print("Selected drones:")

    for name, cf in drones.items():
        print(f"  {name}: {cf.uri}")

    print()
    print("Ctrl+C = EMERGENCY STOP")
    print()


    # --------------------------------------------------------
    # Validate waypoint configuration
    # --------------------------------------------------------

    for name in DRONE_NAMES:

        if name not in WAYPOINTS:
            raise RuntimeError(
                f"No WAYPOINTS configured for {name}"
            )


    number_of_stages = len(
        WAYPOINTS[DRONE_NAMES[0]]
    )


    for name in DRONE_NAMES:

        if len(WAYPOINTS[name]) != number_of_stages:
            raise RuntimeError(
                "All drones must have the same number "
                "of waypoint stages."
            )


    flight_active = False


    try:

        # ====================================================
        # Arm selected drones
        # ====================================================

        print("Arming...")

        for name, cf in drones.items():
            print(f"  arm -> {name}")
            cf.arm(True)

        flight_active = True


        if not wait_interruptible(
            1.0,
            drones,
            time_helper
        ):
            return


        # ====================================================
        # Takeoff
        # ====================================================

        print(
            f"Taking off to {TAKEOFF_HEIGHT:.2f} m..."
        )

        for cf in drones.values():

            cf.takeoff(
                targetHeight=TAKEOFF_HEIGHT,
                duration=TAKEOFF_DURATION
            )


        if not wait_interruptible(
            TAKEOFF_DURATION + 1.0,
            drones,
            time_helper
        ):
            return


        # ====================================================
        # Waypoint stages
        # ====================================================

        for stage in range(number_of_stages):

            print()
            print(
                f"Stage {stage + 1}/{number_of_stages}"
            )


            travel_times = []
            hold_times = []


            # ------------------------------------------------
            # Send one goTo to every drone
            # ------------------------------------------------

            for name, cf in drones.items():

                (
                    position,
                    yaw,
                    travel_time,
                    hold_time
                ) = WAYPOINTS[name][stage]


                print(
                    f"  {name} -> "
                    f"{position}, "
                    f"yaw={math.degrees(yaw):.1f} deg, "
                    f"time={travel_time:.1f}s"
                )


                cf.goTo(
                    goal=position,
                    yaw=yaw,
                    duration=travel_time,
                    relative=False
                )


                travel_times.append(
                    travel_time
                )

                hold_times.append(
                    hold_time
                )


            # ------------------------------------------------
            # Wait for slowest drone
            # ------------------------------------------------

            if not wait_interruptible(
                max(travel_times),
                drones,
                time_helper
            ):
                return


            # ------------------------------------------------
            # Hover
            # ------------------------------------------------

            if not wait_interruptible(
                max(hold_times),
                drones,
                time_helper
            ):
                return


        # ====================================================
        # Landing
        # ====================================================

        print()
        print("Landing...")


        for cf in drones.values():

            cf.land(
                targetHeight=LAND_HEIGHT,
                duration=LAND_DURATION
            )


        if not wait_interruptible(
            LAND_DURATION + 1.0,
            drones,
            time_helper
        ):
            return


        # ====================================================
        # Disarm
        # ====================================================

        print("Disarming...")


        for cf in drones.values():
            cf.arm(False)


        flight_active = False


        wait_interruptible(
            0.5,
            drones,
            time_helper
        )


        print()
        print("Flight complete.")


    except Exception as exc:

        print()
        print("Unexpected error:")
        print(exc)


        if flight_active:

            emergency_stop(
                drones,
                time_helper
            )


        raise


if __name__ == "__main__":
    main()
