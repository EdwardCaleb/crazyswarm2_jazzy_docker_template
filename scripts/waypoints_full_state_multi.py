#!/usr/bin/env python3

import signal

import numpy as np
import rclpy
import transforms3d

from crazyflie_py import Crazyswarm


# ============================================================
# Configuration
# ============================================================

RATE = 30.0

TAKEOFF_HEIGHT = 0.40
TAKEOFF_DURATION = 2.5

LAND_HEIGHT = 0.04
LAND_DURATION = 2.5


# Drones participating in this experiment.
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
# ([x, y, z], travel_time, hold_time)
#
# Each stage starts simultaneously for all drones.
# ============================================================

WAYPOINTS = {

    "cf_1": [
        ([0.20,  0.50, 0.40], 2.0, 1.0),
        ([0.50,  0.50, 0.40], 2.0, 1.0),
        ([0.50,  0.80, 0.40], 2.0, 1.0),
        ([0.20,  0.80, 0.40], 2.0, 1.0),
    ],

    "cf_4": [
        ([0.20,  0.00, 0.40], 2.0, 1.0),
        ([0.50,  0.00, 0.40], 2.0, 1.0),
        ([0.50,  0.30, 0.40], 2.0, 1.0),
        ([0.20,  0.30, 0.40], 2.0, 1.0),
    ],

    "cf_7": [
        ([0.20, -0.50, 0.40], 2.0, 1.0),
        ([0.50, -0.50, 0.40], 2.0, 1.0),
        ([0.50, -0.20, 0.40], 2.0, 1.0),
        ([0.20, -0.20, 0.40], 2.0, 1.0),
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

    # emergency() is asynchronous.
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


# ============================================================
# 7th-order smooth interpolation
# ============================================================

def smooth_segment(p0, p1, t, duration):
    """
    Seventh-order interpolation.

    Boundary conditions:

        velocity     = 0
        acceleration = 0
        jerk         = 0

    at both ends of the segment.
    """

    u = np.clip(
        t / duration,
        0.0,
        1.0
    )

    # Position scaling.
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

    velocity = (
        delta
        * ds
        / duration
    )

    acceleration = (
        delta
        * dds
        / (duration**2)
    )

    return (
        position,
        velocity,
        acceleration
    )


# ============================================================
# Get current yaw
# ============================================================

def get_current_yaw(cf):
    pose = cf.get_pose()

    if not pose:
        raise RuntimeError(
            f"No pose available for {cf.prefix}"
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

    return yaw


# ============================================================
# Execute one synchronized waypoint stage
# ============================================================

def execute_stage(
    drones,
    time_helper,
    start_positions,
    stage_targets,
    yaws,
):
    """
    All drones begin the stage at the same time.

    If one drone finishes its trajectory earlier than another,
    it keeps receiving a hover full-state command until the
    entire stage is complete.
    """

    stage_start_time = time_helper.time()

    # A stage finishes only when every drone has completed
    # its travel time and requested hold time.
    stage_duration = max(
        stage_targets[name]["travel_time"]
        + stage_targets[name]["hold_time"]
        for name in drones
    )

    zero = np.zeros(3)

    while True:

        if abort_requested:
            emergency_stop(
                drones,
                time_helper
            )
            return False

        t = (
            time_helper.time()
            - stage_start_time
        )

        if t >= stage_duration:
            break

        # ----------------------------------------------------
        # Generate and publish one setpoint for every drone
        # during this control cycle.
        # ----------------------------------------------------

        for name, cf in drones.items():

            p0 = start_positions[name]

            target = stage_targets[name]["position"]

            travel_time = stage_targets[name]["travel_time"]

            yaw = yaws[name]

            if t < travel_time:

                position, velocity, acceleration = smooth_segment(
                    p0,
                    target,
                    t,
                    travel_time
                )

            else:

                # This drone already reached its waypoint.
                # Continue streaming a hover setpoint.
                position = target
                velocity = zero
                acceleration = zero

            cf.cmdFullState(
                position,
                velocity,
                acceleration,
                yaw,
                zero
            )

        # One sleep for the whole swarm.
        time_helper.sleepForRate(RATE)


    # --------------------------------------------------------
    # Send the exact final state once more to every drone.
    # --------------------------------------------------------

    for name, cf in drones.items():

        cf.cmdFullState(
            stage_targets[name]["position"],
            zero,
            zero,
            yaws[name],
            zero
        )

    return True


# ============================================================
# Main
# ============================================================

def main():

    swarm = Crazyswarm()

    allcfs = swarm.allcfs
    time_helper = swarm.timeHelper


    # --------------------------------------------------------
    # Ctrl+C handler
    # --------------------------------------------------------

    signal.signal(
        signal.SIGINT,
        sigint_handler
    )


    # --------------------------------------------------------
    # Select drones explicitly by name
    # --------------------------------------------------------

    drones = {}

    for name in DRONE_NAMES:

        if name not in allcfs.crazyfliesByName:
            raise RuntimeError(
                f"{name} was not found. "
                "Check crazyflies.yaml and make sure it is enabled."
            )

        drones[name] = (
            allcfs.crazyfliesByName[name]
        )


    print("Selected drones:")

    for name, cf in drones.items():
        print(
            f"  {name} -> {cf.uri}"
        )

    print()
    print(
        f"cmdFullState rate: {RATE:.1f} Hz"
    )

    print(
        "Ctrl+C = EMERGENCY STOP"
    )

    print()


    # --------------------------------------------------------
    # Validate waypoint configuration
    # --------------------------------------------------------

    for name in DRONE_NAMES:

        if name not in WAYPOINTS:
            raise RuntimeError(
                f"No waypoint list configured for {name}"
            )


    number_of_stages = len(
        WAYPOINTS[DRONE_NAMES[0]]
    )


    for name in DRONE_NAMES:

        if len(WAYPOINTS[name]) != number_of_stages:
            raise RuntimeError(
                "Every drone must currently have "
                "the same number of waypoint stages."
            )


    flight_active = False


    try:

        # ====================================================
        # Arm
        # ====================================================

        print("Arming drones...")

        for name, cf in drones.items():

            print(
                f"  arm -> {name}"
            )

            cf.arm(True)

        flight_active = True


        if not wait_interruptible(
            1.0,
            drones,
            time_helper
        ):
            return


        # ====================================================
        # High-level takeoff
        # ====================================================

        print(
            f"Taking off to "
            f"{TAKEOFF_HEIGHT:.2f} m..."
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


        # Let the pose subscribers process recent messages.
        if not wait_interruptible(
            0.25,
            drones,
            time_helper
        ):
            return


        # ====================================================
        # Capture current positions and yaw
        # ====================================================

        current_positions = {}
        yaws = {}


        print()
        print("Initial full-state conditions:")


        for name, cf in drones.items():

            current_positions[name] = np.array(
                cf.get_position(),
                dtype=float
            )

            yaws[name] = get_current_yaw(
                cf
            )


            print(
                f"  {name}: "
                f"position={current_positions[name]}, "
                f"yaw={np.degrees(yaws[name]):.1f} deg"
            )


        # ====================================================
        # Full-state waypoint stages
        # ====================================================

        for stage in range(
            number_of_stages
        ):

            print()
            print(
                f"Stage "
                f"{stage + 1}/"
                f"{number_of_stages}"
            )


            stage_targets = {}


            for name in DRONE_NAMES:

                (
                    position,
                    travel_time,
                    hold_time
                ) = WAYPOINTS[name][stage]


                stage_targets[name] = {
                    "position": np.array(
                        position,
                        dtype=float
                    ),
                    "travel_time": float(
                        travel_time
                    ),
                    "hold_time": float(
                        hold_time
                    ),
                }


                print(
                    f"  {name}: "
                    f"{current_positions[name]} "
                    f"-> {position}, "
                    f"travel={travel_time:.2f}s, "
                    f"hold={hold_time:.2f}s"
                )


            success = execute_stage(
                drones,
                time_helper,
                current_positions,
                stage_targets,
                yaws
            )


            if not success:
                return


            # The final point of this stage becomes
            # the starting point for the next stage.
            for name in DRONE_NAMES:

                current_positions[name] = (
                    stage_targets[name]["position"]
                )


        # ====================================================
        # Transition from low-level streaming to high-level
        # landing
        # ====================================================

        print()
        print("Full-state trajectory complete.")
        print("Switching to high-level landing...")


        # Tell the firmware that low-level streaming is ending.
        #
        # IMPORTANT:
        # Do NOT sleep between notifySetpointsStop() and land().
        #
        # The final full-state setpoint remains valid briefly,
        # allowing the high-level commander to take over without
        # creating a gap in the commanded state.
        for cf in drones.values():
            cf.notifySetpointsStop()


        # Immediately queue the high-level landing commands.
        for cf in drones.values():

            cf.land(
                targetHeight=LAND_HEIGHT,
                duration=LAND_DURATION
            )


        # Only now do we wait.
        if not wait_interruptible(
            LAND_DURATION + 1.0,
            drones,
            time_helper
        ):
            return


        # ====================================================
        # Disarm
        # ====================================================

        print(
            "Disarming drones..."
        )


        for cf in drones.values():

            cf.arm(False)


        flight_active = False


        wait_interruptible(
            0.5,
            drones,
            time_helper
        )


        print()
        print(
            "Flight complete."
        )


    except Exception as exc:

        print()
        print(
            "Unexpected error:"
        )

        print(exc)


        if flight_active:

            emergency_stop(
                drones,
                time_helper
            )


        raise


if __name__ == "__main__":
    main()
