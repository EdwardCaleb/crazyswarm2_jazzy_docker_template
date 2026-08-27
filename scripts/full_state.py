#!/usr/bin/env python3

"""Unified Crazyswarm2 cmdFullState trajectory runner.

Supports real/sim, one or many drones, absolute waypoints, and geometric
trajectories (circle, sine, square, figure-eight), plus 1D harmonic oscillation.
"""

import argparse
import math
import signal
import numpy as np
import rclpy
import transforms3d
from crazyflie_py import Crazyswarm
RATE = 30.0
TAKEOFF_HEIGHT = 0.4
TAKEOFF_DURATION = 2.5
LAND_HEIGHT = 0.04
LAND_DURATION = 2.5
MIN_TRAJECTORY_Z = 0.08
DEFAULT_DRONES = ['cf_1', 'cf_4', 'cf_7']
WAYPOINTS = {
'cf_1':  [([0.2, 0.5, 0.4], 2.0, 1.0),
          ([0.5, 0.5, 0.4], 2.0, 1.0),
          ([0.5, 0.8, 0.4], 2.0, 1.0),
          ([0.2, 0.8, 0.4], 2.0, 1.0)],
 'cf_4': [([0.2, 0.0, 0.4], 2.0, 1.0),
          ([0.5, 0.0, 0.4], 2.0, 1.0),
          ([0.5, 0.3, 0.4], 2.0, 1.0),
          ([0.2, 0.3, 0.4], 2.0, 1.0)],
 'cf_7': [([0.2, -0.5, 0.4], 2.0, 1.0),
          ([0.5, -0.5, 0.4], 2.0, 1.0),
          ([0.5, -0.2, 0.4], 2.0, 1.0),
          ([0.2, -0.2, 0.4], 2.0, 1.0)]}
abort_requested = False
emergency_sent = False

def parse_args():
    parser = argparse.ArgumentParser(description='Crazyswarm2 multi-drone cmdFullState trajectory generator.')
    parser.add_argument('--mode', choices=['real', 'sim'], required=True)
    parser.add_argument('--trajectory', choices=['waypoints', 'circle', 'sine', 'square', 'figure8', 'oscillate'], required=True)
    parser.add_argument('--drones', nargs='+', default=DEFAULT_DRONES, help='Selected Crazyflie names.')
    parser.add_argument('--plane', choices=['xy', 'xz', 'yz'], default='xy', help='Plane for circle/sine/square/figure8. Ignored for waypoints and oscillate.')
    parser.add_argument('--frequency', type=float, default=0.1, help='Cycles per second [Hz].')
    parser.add_argument('--cycles', type=int, default=1, help='Number of complete cycles/waves.')
    parser.add_argument('--radius', type=float, default=0.25, help='Circle radius [m].')
    parser.add_argument('--side', type=float, default=0.4, help='Square side length [m].')
    parser.add_argument('--amplitude', type=float, default=0.15, help='Amplitude [m] for sine or oscillate.')
    parser.add_argument('--length', type=float, default=0.8, help='Total spatial length of the sine wave [m].')
    parser.add_argument('--distance', type=float, default=0.5, help='Total width of the figure-eight [m].')
    parser.add_argument('--axis', choices=['x', 'y', 'z'], default='x', help='Oscillation axis. Used only by --trajectory oscillate.')
    parser.add_argument('--center', nargs=3, type=float, metavar=('X', 'Y', 'Z'), default=None, help='Absolute formation center [m] for oscillate. If omitted, each drone uses its post-takeoff hover center.')
    parser.add_argument('--entry-duration', type=float, default=2.0, help='Transition to a shape start or requested oscillation center [s].')
    parser.add_argument('--return-duration', type=float, default=2.0, help='Transition from shape end back to hover center [s].')
    parser.add_argument('--hold', type=float, default=1.0, help='Hover at the final shape point before returning [s].')
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error('--cycles must be >= 1')
    if args.trajectory != 'waypoints':
        if args.frequency <= 0.0:
            parser.error('--frequency must be > 0')
        if args.entry_duration <= 0.0:
            parser.error('--entry-duration must be > 0')
        if args.return_duration <= 0.0:
            parser.error('--return-duration must be > 0')
        if args.hold < 0.0:
            parser.error('--hold must be >= 0')
    if args.trajectory == 'circle' and args.radius <= 0.0:
        parser.error('--radius must be > 0')
    if args.trajectory == 'square' and args.side <= 0.0:
        parser.error('--side must be > 0')
    if args.trajectory == 'sine':
        if args.amplitude <= 0.0:
            parser.error('--amplitude must be > 0')
        if args.length <= 0.0:
            parser.error('--length must be > 0')
    if args.trajectory == 'figure8' and args.distance <= 0.0:
        parser.error('--distance must be > 0')
    if args.trajectory == 'oscillate' and args.amplitude <= 0.0:
        parser.error('--amplitude must be > 0')
    return args

def sigint_handler(signum, frame):
    global abort_requested
    if not abort_requested:
        print('\nCTRL+C -> abort requested')
    abort_requested = True

def emergency_stop_real(drones, time_helper):
    global emergency_sent
    if emergency_sent:
        return
    emergency_sent = True
    print('\n========================================')
    print('!!! EMERGENCY STOP - SELECTED DRONES !!!')
    print('========================================')
    for name, cf in drones.items():
        print(f'Emergency -> {name}')
        cf.emergency()
    end_time = time_helper.time() + 0.5
    while rclpy.ok() and time_helper.time() < end_time:
        rclpy.spin_once(time_helper.node, timeout_sec=0.01)

def wait_interruptible(duration, drones, time_helper, mode):
    end_time = time_helper.time() + duration
    while time_helper.time() < end_time:
        if abort_requested:
            if mode == 'real':
                emergency_stop_real(drones, time_helper)
            return False
        rclpy.spin_once(time_helper.node, timeout_sec=0.01)
    return True

def smooth_profile(t, duration):
    """
    Returns s(t), ds/dt and d2s/dt2.

    At t=0 and t=T:

        velocity     = 0
        acceleration = 0
        jerk         = 0
    """
    u = np.clip(t / duration, 0.0, 1.0)
    s = 35.0 * u ** 4 - 84.0 * u ** 5 + 70.0 * u ** 6 - 20.0 * u ** 7
    ds_du = 140.0 * u ** 3 - 420.0 * u ** 4 + 420.0 * u ** 5 - 140.0 * u ** 6
    d2s_du2 = 420.0 * u ** 2 - 1680.0 * u ** 3 + 2100.0 * u ** 4 - 840.0 * u ** 5
    return (s, ds_du / duration, d2s_du2 / duration ** 2)

def smooth_segment(p0, p1, t, duration):
    s, s_dot, s_ddot = smooth_profile(t, duration)
    delta = p1 - p0
    position = p0 + delta * s
    velocity = delta * s_dot
    acceleration = delta * s_ddot
    return (position, velocity, acceleration)

def get_real_initial_states(drones):
    positions = {}
    yaws = {}
    for name, cf in drones.items():
        positions[name] = np.array(cf.get_position(), dtype=float)
        pose = cf.get_pose()
        if not pose:
            raise RuntimeError(f'No pose available for {name}')
        q = pose['orientation']
        _, _, yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
        yaws[name] = yaw
    return (positions, yaws)

def get_sim_initial_states(drones):
    positions = {}
    yaws = {}
    for name, cf in drones.items():
        position = np.array(cf.initialPosition, dtype=float)
        position[2] += TAKEOFF_HEIGHT
        positions[name] = position
        yaws[name] = 0.0
    return (positions, yaws)

def plane_indices(plane):
    return {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}[plane]

def embed_2d_state(center, plane, uv, uv_dot, uv_ddot):
    i, j = plane_indices(plane)
    position = center.copy()
    velocity = np.zeros(3)
    acceleration = np.zeros(3)
    position[i] += uv[0]
    position[j] += uv[1]
    velocity[i] = uv_dot[0]
    velocity[j] = uv_dot[1]
    acceleration[i] = uv_ddot[0]
    acceleration[j] = uv_ddot[1]
    return (position, velocity, acceleration)

def shape_duration(args):
    return args.cycles / args.frequency

def shape_start_offset(args):
    if args.trajectory == 'circle':
        return np.array([args.radius, 0.0])
    if args.trajectory == 'square':
        half = args.side / 2.0
        return np.array([-half, -half])
    if args.trajectory == 'sine':
        return np.array([-args.length / 2.0, 0.0])
    if args.trajectory == 'figure8':
        return np.zeros(2)
    raise ValueError(f'No shape start for {args.trajectory}')

def phase_state(t, duration, cycles):
    """
    Smooth phase from 0 to 2*pi*cycles.

    This prevents an instantaneous tangential velocity at the
    beginning/end of circle and figure-eight trajectories.
    """
    s, s_dot, s_ddot = smooth_profile(t, duration)
    total_phase = 2.0 * math.pi * cycles
    theta = total_phase * s
    theta_dot = total_phase * s_dot
    theta_ddot = total_phase * s_ddot
    return (theta, theta_dot, theta_ddot)

def circle_state_2d(t, duration, args):
    theta, theta_dot, theta_ddot = phase_state(t, duration, args.cycles)
    r = args.radius
    c = math.cos(theta)
    s = math.sin(theta)
    uv = np.array([r * c, r * s])
    d_uv_dtheta = np.array([-r * s, r * c])
    d2_uv_dtheta2 = np.array([-r * c, -r * s])
    uv_dot = d_uv_dtheta * theta_dot
    uv_ddot = d2_uv_dtheta2 * theta_dot ** 2 + d_uv_dtheta * theta_ddot
    return (uv, uv_dot, uv_ddot)

def sine_state_2d(t, duration, args):
    """
    Spatial sine wave.

    u moves from -length/2 to +length/2.
    v oscillates with the requested amplitude.
    """
    s, s_dot, s_ddot = smooth_profile(t, duration)
    theta, theta_dot, theta_ddot = phase_state(t, duration, args.cycles)
    u = -args.length / 2.0 + args.length * s
    u_dot = args.length * s_dot
    u_ddot = args.length * s_ddot
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    v = args.amplitude * sin_theta
    v_dot = args.amplitude * cos_theta * theta_dot
    v_ddot = args.amplitude * (-sin_theta * theta_dot ** 2 + cos_theta * theta_ddot)
    return (np.array([u, v]), np.array([u_dot, v_dot]), np.array([u_ddot, v_ddot]))

def figure8_state_2d(t, duration, args):
    """
    Gerono-style figure eight.

    --distance is the total width.
    Total height is distance / 2.
    """
    theta, theta_dot, theta_ddot = phase_state(t, duration, args.cycles)
    a = args.distance / 2.0
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    sin_2theta = math.sin(2.0 * theta)
    cos_2theta = math.cos(2.0 * theta)
    uv = np.array([a * sin_theta, 0.5 * a * sin_2theta])
    d_uv_dtheta = np.array([a * cos_theta, a * cos_2theta])
    d2_uv_dtheta2 = np.array([-a * sin_theta, -2.0 * a * sin_2theta])
    uv_dot = d_uv_dtheta * theta_dot
    uv_ddot = d2_uv_dtheta2 * theta_dot ** 2 + d_uv_dtheta * theta_ddot
    return (uv, uv_dot, uv_ddot)

def square_state_2d(t, duration, args):
    """
    Square made from four seventh-order smooth segments.

    Velocity, acceleration and jerk go to zero at every corner.
    """
    half = args.side / 2.0
    corners = [np.array([-half, -half]), np.array([half, -half]), np.array([half, half]), np.array([-half, half])]
    total_sides = 4 * args.cycles
    side_duration = duration / total_sides
    if t >= duration:
        return (corners[0].copy(), np.zeros(2), np.zeros(2))
    side_number = min(int(t / side_duration), total_sides - 1)
    local_time = t - side_number * side_duration
    corner_index = side_number % 4
    next_corner_index = (corner_index + 1) % 4
    return smooth_segment(corners[corner_index], corners[next_corner_index], local_time, side_duration)

def shape_state_2d(t, duration, args):
    if args.trajectory == 'circle':
        return circle_state_2d(t, duration, args)
    if args.trajectory == 'sine':
        return sine_state_2d(t, duration, args)
    if args.trajectory == 'square':
        return square_state_2d(t, duration, args)
    if args.trajectory == 'figure8':
        return figure8_state_2d(t, duration, args)
    raise ValueError(f'Unsupported trajectory: {args.trajectory}')

def vertical_extent(args):
    """Maximum negative v excursion for vertical planes."""
    if args.trajectory == 'circle':
        return args.radius
    if args.trajectory == 'square':
        return args.side / 2.0
    if args.trajectory == 'sine':
        return args.amplitude
    if args.trajectory == 'figure8':
        return args.distance / 4.0
    return 0.0

def oscillation_centers(centers, args):
    """Return per-drone centers for harmonic oscillation.

    With no --center, every drone oscillates around its own post-takeoff
    hover point.

    With --center X Y Z, that point becomes the center of the whole
    formation. Relative offsets between drones are preserved, so multiple
    drones are never commanded to converge onto the same point.
    """
    if args.center is None:
        return {name: center.copy() for name, center in centers.items()}

    requested_center = np.array(args.center, dtype=float)
    formation_center = np.mean(np.stack(list(centers.values())), axis=0)

    return {
        name: requested_center + (center - formation_center)
        for name, center in centers.items()
    }


def oscillation_envelope(t, duration):
    """Smooth amplitude envelope for the harmonic oscillator.

    The oscillator keeps a fixed angular frequency omega=2*pi*f, while
    the amplitude ramps smoothly from 0 to 1 and back to 0 over the first
    and last 15% of the motion. This avoids an instantaneous velocity or
    acceleration jump when low-level streaming starts or stops.

    Returns envelope, envelope_dot, envelope_ddot.
    """
    ramp_duration = 0.15 * duration

    if t < ramp_duration:
        return smooth_profile(t, ramp_duration)

    if t > duration - ramp_duration:
        tau = duration - t
        envelope, d_envelope_d_tau, d2_envelope_d_tau2 = smooth_profile(
            tau, ramp_duration
        )
        return envelope, -d_envelope_d_tau, d2_envelope_d_tau2

    return 1.0, 0.0, 0.0


def oscillation_state_1d(t, duration, args):
    """Simple harmonic motion with smooth amplitude ramp-up/down.

        q(t) = A E(t) sin(omega t)

    During the middle of the trajectory E(t)=1, so this is ordinary
    simple harmonic motion with the exact requested frequency.
    """
    omega = 2.0 * math.pi * args.frequency
    theta = omega * t

    envelope, envelope_dot, envelope_ddot = oscillation_envelope(
        t, duration
    )

    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)

    position = args.amplitude * envelope * sin_theta

    velocity = args.amplitude * (
        envelope_dot * sin_theta
        + envelope * omega * cos_theta
    )

    acceleration = args.amplitude * (
        envelope_ddot * sin_theta
        + 2.0 * envelope_dot * omega * cos_theta
        - envelope * omega**2 * sin_theta
    )

    return position, velocity, acceleration


def execute_oscillation(drones, time_helper, centers, yaws, args):
    """Execute synchronized 1D harmonic motion for all selected drones."""
    target_centers = oscillation_centers(centers, args)

    if any(
        not np.allclose(target_centers[name], centers[name])
        for name in drones
    ):
        print('Moving formation to oscillation center...')
        if not stream_transition(
            drones,
            time_helper,
            centers,
            target_centers,
            yaws,
            args.entry_duration,
            args.mode,
        ):
            return False

    duration = shape_duration(args)
    axis_index = {'x': 0, 'y': 1, 'z': 2}[args.axis]

    print(
        f'Executing oscillation: axis={args.axis}, '
        f'amplitude={args.amplitude:.3f} m, '
        f'frequency={args.frequency:.3f} Hz, '
        f'cycles={args.cycles}, duration={duration:.2f} s'
    )

    start_time = time_helper.time()
    zero3 = np.zeros(3)

    while True:
        if abort_requested:
            if args.mode == 'real':
                emergency_stop_real(drones, time_helper)
            return False

        t = time_helper.time() - start_time
        if t >= duration:
            break

        q, q_dot, q_ddot = oscillation_state_1d(t, duration, args)

        for name, cf in drones.items():
            position = target_centers[name].copy()
            velocity = zero3.copy()
            acceleration = zero3.copy()

            position[axis_index] += q
            velocity[axis_index] = q_dot
            acceleration[axis_index] = q_ddot

            cf.cmdFullState(
                position,
                velocity,
                acceleration,
                yaws[name],
                zero3,
            )

        time_helper.sleepForRate(RATE)

    # The envelope is exactly zero at the end, so the final state is the
    # oscillation center with zero velocity and acceleration.
    for name, cf in drones.items():
        cf.cmdFullState(
            target_centers[name],
            zero3,
            zero3,
            yaws[name],
            zero3,
        )

    if not stream_hold(
        drones,
        time_helper,
        target_centers,
        yaws,
        args.hold,
        args.mode,
    ):
        return False

    if any(
        not np.allclose(target_centers[name], centers[name])
        for name in drones
    ):
        print('Returning formation to original hover centers...')
        if not stream_transition(
            drones,
            time_helper,
            target_centers,
            centers,
            yaws,
            args.return_duration,
            args.mode,
        ):
            return False

    return True


def unsafe_oscillation(centers, args):
    """Check Z safety for oscillation center/transition and z-axis motion."""
    target_centers = oscillation_centers(centers, args)
    unsafe = []

    for name in centers:
        # The transition is a convex straight segment, so the minimum Z is
        # one of its endpoints.
        minimum_z = min(centers[name][2], target_centers[name][2])

        if args.axis == 'z':
            minimum_z = min(
                minimum_z,
                target_centers[name][2] - args.amplitude,
            )

        if minimum_z < MIN_TRAJECTORY_Z:
            unsafe.append((name, minimum_z))

    return unsafe


def stream_transition(drones, time_helper, start_positions, target_positions, yaws, duration, mode):
    start_time = time_helper.time()
    zero = np.zeros(3)
    while True:
        if abort_requested:
            if mode == 'real':
                emergency_stop_real(drones, time_helper)
            return False
        t = time_helper.time() - start_time
        if t >= duration:
            break
        for name, cf in drones.items():
            position, velocity, acceleration = smooth_segment(start_positions[name], target_positions[name], t, duration)
            cf.cmdFullState(position, velocity, acceleration, yaws[name], zero)
        time_helper.sleepForRate(RATE)
    for name, cf in drones.items():
        cf.cmdFullState(target_positions[name], zero, zero, yaws[name], zero)
    return True

def stream_hold(drones, time_helper, positions, yaws, duration, mode):
    if duration <= 0.0:
        return True
    start_time = time_helper.time()
    zero = np.zeros(3)
    while time_helper.time() - start_time < duration:
        if abort_requested:
            if mode == 'real':
                emergency_stop_real(drones, time_helper)
            return False
        for name, cf in drones.items():
            cf.cmdFullState(positions[name], zero, zero, yaws[name], zero)
        time_helper.sleepForRate(RATE)
    return True

def execute_shape(drones, time_helper, centers, yaws, args):
    zero2 = np.zeros(2)
    start_offset = shape_start_offset(args)
    start_positions = {}
    for name in drones:
        position, _, _ = embed_2d_state(centers[name], args.plane, start_offset, zero2, zero2)
        start_positions[name] = position
    if not np.allclose(start_offset, zero2):
        print(f'Moving to {args.trajectory} start point...')
        if not stream_transition(drones, time_helper, centers, start_positions, yaws, args.entry_duration, args.mode):
            return False
    duration = shape_duration(args)
    print(f'Executing {args.trajectory}: {args.cycles} cycle(s), {args.frequency:.3f} Hz, duration={duration:.2f} s, plane={args.plane}')
    start_time = time_helper.time()
    zero3 = np.zeros(3)
    while True:
        if abort_requested:
            if args.mode == 'real':
                emergency_stop_real(drones, time_helper)
            return False
        t = time_helper.time() - start_time
        if t >= duration:
            break
        uv, uv_dot, uv_ddot = shape_state_2d(t, duration, args)
        for name, cf in drones.items():
            position, velocity, acceleration = embed_2d_state(centers[name], args.plane, uv, uv_dot, uv_ddot)
            cf.cmdFullState(position, velocity, acceleration, yaws[name], zero3)
        time_helper.sleepForRate(RATE)
    uv, _, _ = shape_state_2d(duration, duration, args)
    final_positions = {}
    for name, cf in drones.items():
        position, _, _ = embed_2d_state(centers[name], args.plane, uv, zero2, zero2)
        final_positions[name] = position
        cf.cmdFullState(position, zero3, zero3, yaws[name], zero3)
    if not stream_hold(drones, time_helper, final_positions, yaws, args.hold, args.mode):
        return False
    needs_return = any((not np.allclose(final_positions[name], centers[name]) for name in drones))
    if needs_return:
        print('Returning to hover centers...')
        if not stream_transition(drones, time_helper, final_positions, centers, yaws, args.return_duration, args.mode):
            return False
    return True

def execute_waypoints(drones, time_helper, current_positions, yaws, mode):
    drone_names = list(drones.keys())
    for name in drone_names:
        if name not in WAYPOINTS:
            raise RuntimeError(f'No WAYPOINTS configured for {name}')
    number_of_stages = len(WAYPOINTS[drone_names[0]])
    for name in drone_names:
        if len(WAYPOINTS[name]) != number_of_stages:
            raise RuntimeError('Every selected drone must have the same number of waypoint stages.')
        for position, _, _ in WAYPOINTS[name]:
            if position[2] < MIN_TRAJECTORY_Z:
                raise RuntimeError(f'Unsafe waypoint Z for {name}: {position[2]:.3f} m')
    zero = np.zeros(3)
    for stage in range(number_of_stages):
        print(f'\nWaypoint stage {stage + 1}/{number_of_stages}')
        stage_targets = {}
        for name in drone_names:
            position, travel_time, hold_time = WAYPOINTS[name][stage]
            stage_targets[name] = {'position': np.array(position, dtype=float), 'travel_time': float(travel_time), 'hold_time': float(hold_time)}
            print(f"  {name}: {current_positions[name]} -> {stage_targets[name]['position']} | travel={travel_time:.2f}s | hold={hold_time:.2f}s")
        stage_duration = max((stage_targets[name]['travel_time'] + stage_targets[name]['hold_time'] for name in drone_names))
        start_time = time_helper.time()
        while True:
            if abort_requested:
                if mode == 'real':
                    emergency_stop_real(drones, time_helper)
                return False
            t = time_helper.time() - start_time
            if t >= stage_duration:
                break
            for name, cf in drones.items():
                target = stage_targets[name]['position']
                travel_time = stage_targets[name]['travel_time']
                if t < travel_time:
                    position, velocity, acceleration = smooth_segment(current_positions[name], target, t, travel_time)
                else:
                    position = target
                    velocity = zero
                    acceleration = zero
                cf.cmdFullState(position, velocity, acceleration, yaws[name], zero)
            time_helper.sleepForRate(RATE)
        for name, cf in drones.items():
            target = stage_targets[name]['position']
            cf.cmdFullState(target, zero, zero, yaws[name], zero)
            current_positions[name] = target.copy()
    return True

def validate_waypoints_config(drone_names):
    if not drone_names:
        raise RuntimeError("At least one drone must be selected.")
    number_of_stages = None
    for name in drone_names:
        if name not in WAYPOINTS:
            raise RuntimeError(f"No WAYPOINTS configured for {name}")
        if number_of_stages is None:
            number_of_stages = len(WAYPOINTS[name])
        elif len(WAYPOINTS[name]) != number_of_stages:
            raise RuntimeError("Every selected drone must have the same number of waypoint stages.")
        for position, travel_time, hold_time in WAYPOINTS[name]:
            if position[2] < MIN_TRAJECTORY_Z:
                raise RuntimeError(f"Unsafe waypoint Z for {name}: {position[2]:.3f} m")
            if travel_time <= 0.0:
                raise RuntimeError(f"Waypoint travel_time must be > 0 for {name}")
            if hold_time < 0.0:
                raise RuntimeError(f"Waypoint hold_time must be >= 0 for {name}")

def describe_trajectory(args):
    if args.trajectory == 'circle':
        return f'circle radius={args.radius:.3f} m'
    if args.trajectory == 'square':
        return f'square side={args.side:.3f} m'
    if args.trajectory == 'sine':
        return f'sine amplitude={args.amplitude:.3f} m, length={args.length:.3f} m'
    if args.trajectory == 'figure8':
        return f'figure8 width={args.distance:.3f} m'
    if args.trajectory == 'oscillate':
        center = 'post-takeoff centers' if args.center is None else f'formation center={args.center}'
        return f'oscillate axis={args.axis}, amplitude={args.amplitude:.3f} m, {center}'
    return 'absolute per-drone waypoints'

def unsafe_vertical_shape(centers, args):
    if args.trajectory == 'waypoints':
        return []
    if args.trajectory == 'oscillate':
        return unsafe_oscillation(centers, args)
    if args.plane == 'xy':
        return []
    extent = vertical_extent(args)
    unsafe = []
    for name, center in centers.items():
        minimum_z = center[2] - extent
        if minimum_z < MIN_TRAJECTORY_Z:
            unsafe.append((name, minimum_z))
    return unsafe

def main():
    args = parse_args()
    if len(set(args.drones)) != len(args.drones):
        raise RuntimeError('--drones contains duplicate names')
    if args.trajectory == 'waypoints':
        validate_waypoints_config(args.drones)
    print('\n========================================')
    print('Crazyswarm2 Multi Full-State Trajectory')
    print('========================================')
    print(f'Mode:       {args.mode}')
    print(f"Drones:     {', '.join(args.drones)}")
    print(f'Trajectory: {describe_trajectory(args)}')
    if args.trajectory not in ('waypoints', 'oscillate'):
        print(f'Plane:      {args.plane}')
    if args.trajectory == 'oscillate':
        print(f'Axis:       {args.axis}')
        if args.center is not None:
            print(f'Center:     {args.center}')
    if args.trajectory != 'waypoints':
        print(f'Frequency:  {args.frequency:.3f} Hz')
        print(f'Cycles:     {args.cycles}')
    print(f'Rate:       {RATE:.1f} Hz')
    print()
    swarm = Crazyswarm()
    allcfs = swarm.allcfs
    time_helper = swarm.timeHelper
    signal.signal(signal.SIGINT, sigint_handler)
    drones = {}
    for name in args.drones:
        if name not in allcfs.crazyfliesByName:
            raise RuntimeError(f'{name} was not found. Check crazyflies.yaml.')
        drones[name] = allcfs.crazyfliesByName[name]
    print('Selected drones:')
    for name, cf in drones.items():
        print(f'  {name} -> {cf.uri}')
    flight_active = False
    try:
        if args.mode == 'real':
            print('\nArming drones...')
            for cf in drones.values():
                cf.arm(True)
            flight_active = True
            if not wait_interruptible(1.0, drones, time_helper, args.mode):
                return
        print(f'\nTaking off to {TAKEOFF_HEIGHT:.2f} m...')
        for cf in drones.values():
            cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        if not wait_interruptible(TAKEOFF_DURATION + 1.0, drones, time_helper, args.mode):
            return
        if args.mode == 'real':
            centers, yaws = get_real_initial_states(drones)
        else:
            centers, yaws = get_sim_initial_states(drones)
        print('\nFull-state centers:')
        for name in drones:
            print(f'  {name}: position={centers[name]}, yaw={math.degrees(yaws[name]):.1f} deg')
        unsafe = unsafe_vertical_shape(centers, args)
        if unsafe:
            print('\nTrajectory rejected: vertical path goes too low.')
            for name, minimum_z in unsafe:
                print(f'  {name}: planned minimum z={minimum_z:.3f} m')
            print('Increase TAKEOFF_HEIGHT or reduce the shape size.')
            for cf in drones.values():
                cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
            wait_interruptible(LAND_DURATION + 1.0, drones, time_helper, args.mode)
            if args.mode == 'real':
                for cf in drones.values():
                    cf.arm(False)
                flight_active = False
            return
        if args.trajectory == 'waypoints':
            current_positions = {name: centers[name].copy() for name in drones}
            success = execute_waypoints(drones, time_helper, current_positions, yaws, args.mode)
        elif args.trajectory == 'oscillate':
            success = execute_oscillation(drones, time_helper, centers, yaws, args)
        else:
            success = execute_shape(drones, time_helper, centers, yaws, args)
        if not success:
            return
        print('\nFull-state trajectory complete.')
        print('Switching to high-level landing...')
        for cf in drones.values():
            cf.notifySetpointsStop(remainValidMillisecs=200)
        for cf in drones.values():
            cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
        if not wait_interruptible(LAND_DURATION + 1.0, drones, time_helper, args.mode):
            return
        if args.mode == 'real':
            print('Disarming drones...')
            for cf in drones.values():
                cf.arm(False)
            flight_active = False
            wait_interruptible(0.5, drones, time_helper, args.mode)
        print(f'\n{args.mode.upper()} trajectory complete.')
    except Exception as exc:
        print('\nUnexpected error:')
        print(exc)
        if args.mode == 'real' and flight_active:
            emergency_stop_real(drones, time_helper)
        raise
if __name__ == '__main__':
    main()