# Crazyswarm2 + ROS 2 Jazzy Docker Template

Docker-based development environment for **Crazyswarm2**, **ROS 2 Jazzy**, **Crazyradio PA**, **OptiTrack/Motive**, **RViz2**, and the Crazyswarm2 software-in-the-loop simulator.

The project uses the same robot configuration and trajectory scripts for both **real flights** and **simulation**.

## Features

- ROS 2 Jazzy on Ubuntu 24.04
- Crazyswarm2 pinned to an exact commit
- Crazyflie firmware Python bindings for Crazyswarm2 simulation
- Crazyradio PA USB access from Docker
- OptiTrack/Motive motion capture
- RViz2 inside Docker
- Real and simulation launch modes
- Single- and multi-drone `goTo()` experiments
- Unified multi-drone `cmdFullState()` trajectory runner
- Smooth seventh-order waypoint interpolation
- Circle, square, sine-wave, figure-eight, and harmonic oscillation trajectories
- `xy`, `xz`, and `yz` trajectory planes
- `Ctrl+C` emergency stop for real-flight full-state experiments

## Project layout

```text
crazyswarm2_jazzy_docker_template/
├── .env
├── compose.yaml
├── config/
│   ├── crazyflies.yaml
│   ├── crazyflies.yaml.upstream
│   └── motion_capture.yaml
├── docker/
│   ├── Dockerfile
│   ├── requirements-python.txt
│   ├── start.sh
│   └── shell.sh
├── scripts/
│   ├── waypoints_goto.py
│   ├── waypoints_multi_drones.py
│   └── full_state.py
└── ros2_ws/
    └── src/
```

## 1. Requirements

- Ubuntu 24.04
- Docker Engine
- Docker Compose
- X11 desktop session for RViz2

For real flight:

- Crazyradio PA
- Crazyflie/Bolt vehicles with unique radio URIs
- OptiTrack Motive reachable from the Ubuntu host

Simulation does not require Crazyradio, OptiTrack, or powered vehicles.

## 2. Pinned upstream versions

The `.env` file pins both Crazyswarm2 and the Crazyflie firmware used for the simulator:

```bash
CRAZYSWARM2_REF=fcf51e27fe675bbdb564a40f4ca1ee4336c062ea
CRAZYFLIE_FIRMWARE_REF=16c5630707e92830dcd16455a57b5560e5838735
```

This keeps the Docker image reproducible.

## 3. Configure the robots

Edit:

```bash
nano config/crazyflies.yaml
```

Example:

```yaml
robots:
  cf_1:
    enabled: true
    uri: radio://0/80/2M/E7E7E7EA01
    initial_position: [0.0, 0.5, 0.0]
    type: bolt_3in_2s

  cf_4:
    enabled: true
    uri: radio://0/80/2M/XXXXXXXXXX
    initial_position: [0.0, 0.0, 0.0]
    type: bolt_3in_2s

  cf_7:
    enabled: true
    uri: radio://0/80/2M/YYYYYYYYYY
    initial_position: [0.0, -0.5, 0.0]
    type: bolt_3in_2s
```

The same `config/crazyflies.yaml` is used by both the real and simulation backends.

`initial_position` is especially important in simulation because it determines where each simulated drone starts. For real `cmdFullState()` experiments, the scripts use the measured pose after takeoff.

The current Bolt configuration uses the high-level commander, Kalman estimator, and PID controller:

```yaml
firmware_params:
  commander:
    enHighLevel: 1
  stabilizer:
    estimator: 2
    controller: 1
  locSrv:
    extPosStdDev: 1e-3
    extQuatStdDev: 0.05
```

## 4. Configure OptiTrack

Real-flight mode uses:

```text
config/motion_capture.yaml
```

Set the Motive server address, for example:

```yaml
/motion_capture_tracking:
  ros__parameters:
    type: "optitrack"
    hostname: "192.168.0.4"
```

Rigid-body names in Motive must match the robot names in `crazyflies.yaml`, for example:

```text
cf_1
cf_4
cf_7
```

Simulation does not launch the motion-capture node.

## 5. Build the Docker image

```bash
source .env

docker build \
  --build-arg CRAZYSWARM2_REF="$CRAZYSWARM2_REF" \
  --build-arg CRAZYFLIE_FIRMWARE_REF="$CRAZYFLIE_FIRMWARE_REF" \
  -t crazyswarm2:jazzy-pinned \
  -f docker/Dockerfile \
  .
```

The image builds Crazyswarm2 and also compiles the Crazyflie firmware Python bindings required by `crazyflie_sim`.

## 6. Helper scripts

Make the helper scripts executable once:

```bash
chmod +x docker/start.sh docker/shell.sh
```

### Real hardware

```bash
./docker/start.sh --real
```

This starts:

```text
backend = cflib
mocap   = True
rviz    = True
```

In RViz2, use:

```text
Fixed Frame = mocap
```

### Simulation

```bash
./docker/start.sh --sim
```

This starts:

```text
backend = sim
mocap   = False
rviz    = True
```

In RViz2, use:

```text
Fixed Frame = world
```

Both modes use:

```text
config/crazyflies.yaml
```

## 7. Open a ROS shell

With either mode running:

```bash
./docker/shell.sh
```

This opens a shell inside the active `crazyswarm2_dev` container and sources ROS 2 and Crazyswarm2.

Example checks:

```bash
ros2 node list
ros2 topic list
```

For real OptiTrack operation:

```bash
ros2 topic echo /poses --once
```

## 8. Basic real-flight commands

Arm all enabled drones:

```bash
ros2 service call /all/arm \
  crazyflie_interfaces/srv/Arm \
  "{arm: true}"
```

Take off:

```bash
ros2 service call /all/takeoff \
  crazyflie_interfaces/srv/Takeoff \
  "{group_mask: 0, height: 0.30, duration: {sec: 3, nanosec: 0}}"
```

Land:

```bash
ros2 service call /all/land \
  crazyflie_interfaces/srv/Land \
  "{group_mask: 0, height: 0.04, duration: {sec: 3, nanosec: 0}}"
```

Disarm:

```bash
ros2 service call /all/arm \
  crazyflie_interfaces/srv/Arm \
  "{arm: false}"
```

## 9. `goTo()` waypoint experiments

Single drone:

```bash
python3 /workspace/scripts/waypoints_goto.py
```

Multiple drones:

```bash
python3 /workspace/scripts/waypoints_multi_drones.py
```

These use the Crazyflie high-level commander, which generates the trajectory onboard.

## 10. Unified `cmdFullState()` trajectory runner

The main full-state experiment is:

```text
scripts/full_state.py
```

It supports both real and simulated execution:

```bash
python3 /workspace/scripts/full_state.py --mode real ...
```

or:

```bash
python3 /workspace/scripts/full_state.py --mode sim ...
```

Select one or more drones with:

```bash
--drones cf_1 cf_4 cf_7
```

The script streams full-state setpoints at 30 Hz:

```text
position
velocity
acceleration
yaw
angular velocity
```

### Waypoints

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory waypoints \
  --drones cf_1 cf_4 cf_7
```

Waypoint segments use seventh-order interpolation with zero velocity, acceleration, and jerk at the segment boundaries.

### Circle

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory circle \
  --drones cf_1 cf_4 cf_7 \
  --plane xy \
  --radius 0.25 \
  --frequency 0.10 \
  --cycles 2
```

### Square

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory square \
  --drones cf_1 cf_4 cf_7 \
  --plane xy \
  --side 0.40 \
  --frequency 0.10 \
  --cycles 1
```

### Spatial sine wave

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory sine \
  --drones cf_1 cf_4 cf_7 \
  --plane xy \
  --amplitude 0.15 \
  --length 0.80 \
  --frequency 0.15 \
  --cycles 2
```

This is a spatial wave: one trajectory coordinate advances while the other follows a sinusoidal shape.

### Figure eight

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory figure8 \
  --drones cf_1 cf_4 cf_7 \
  --plane xy \
  --distance 0.50 \
  --frequency 0.10 \
  --cycles 2
```

### Harmonic oscillation

Use `oscillate` for simple harmonic motion around a point:

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory oscillate \
  --drones cf_1 cf_4 cf_7 \
  --axis x \
  --amplitude 0.15 \
  --frequency 0.25 \
  --cycles 3
```

The oscillation can use:

```text
--axis x
--axis y
--axis z
```

Optionally choose a center:

```bash
--center 0.5 0.0 0.5
```

For multiple drones, `--center` translates the center of the formation while preserving the relative spacing between drones.

## 11. Trajectory planes

The 2D trajectories support:

```text
--plane xy
--plane xz
--plane yz
```

For example:

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory circle \
  --drones cf_1 \
  --plane xz \
  --radius 0.10 \
  --frequency 0.10
```

The script checks the planned minimum Z before entering full-state streaming and rejects trajectories that would go below the configured safe minimum altitude.

## 12. Test in simulation first

A typical workflow is:

Terminal 1:

```bash
./docker/start.sh --sim
```

Terminal 2:

```bash
./docker/shell.sh
```

Then:

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory figure8 \
  --drones cf_1 cf_4 cf_7 \
  --plane xy \
  --distance 0.50 \
  --frequency 0.10 \
  --cycles 2
```

After validating the geometry, timing, spacing, and altitude in RViz2, launch the real stack:

```bash
./docker/start.sh --real
```

and run the same trajectory with:

```bash
--mode real
```

Simulation is useful for trajectory validation, but it is not a complete physical model of a custom Bolt + 3-inch propeller + 2S vehicle. A trajectory that works in simulation still needs conservative real-flight testing.

## 13. Full-state control transition

During a full-state trajectory the script continuously streams setpoints.

At the end it transitions back to the high-level commander using:

```text
last cmdFullState
       ↓
notifySetpointsStop(remainValidMillisecs=200)
       ↓
land() immediately
```

There is intentionally no delay between `notifySetpointsStop()` and `land()`. This avoids a gap in commanded state during the transition.

## 14. Emergency stop

For real hardware, while a full-state experiment is running:

```text
Ctrl+C
```

requests an emergency stop for the selected drones.

You can also call:

```bash
ros2 service call /all/emergency \
  std_srvs/srv/Empty \
  "{}"
```

Emergency stop cuts motor power immediately. It is not a controlled landing, and the vehicles must be rebooted before normal flight resumes.

A firmware-side radio-loss watchdog is not yet integrated into this template.

## 15. Recommended workflow

### Simulation

Terminal 1:

```bash
./docker/start.sh --sim
```

Terminal 2:

```bash
./docker/shell.sh
```

Then run:

```bash
python3 /workspace/scripts/full_state.py \
  --mode sim \
  --trajectory <trajectory> \
  --drones <names>
```

### Real flight

Terminal 1:

```bash
./docker/start.sh --real
```

Terminal 2:

```bash
./docker/shell.sh
```

Then run the same experiment with:

```bash
python3 /workspace/scripts/full_state.py \
  --mode real \
  --trajectory <trajectory> \
  --drones <names>
```

## Safety

Use simulation first, keep initial real-flight trajectories slow and spatially separated, and verify that complete path segments do not intersect.

`Ctrl+C` emergency handling still depends on a working ROS → Crazyradio → Crazyflie communication path. It does not replace an onboard radio-loss watchdog.
