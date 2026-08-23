# Crazyswarm2 + ROS 2 Jazzy Docker Template

Minimal Docker setup for running **Crazyswarm2**, **ROS 2 Jazzy**, **OptiTrack/Motive**, **Crazyradio PA**, **RViz2**, and custom Crazyflie/Bolt waypoint experiments on Ubuntu 24.04.

## Requirements

- Ubuntu 24.04
- Docker Engine + Docker Compose
- Crazyradio PA
- OptiTrack Motive on the same network as the host
- X11 desktop session for RViz2
- Crazyflie/Bolt vehicles configured with unique radio URIs

## 1. Clone

```bash
git clone <YOUR_REPOSITORY_URL>
cd crazyswarm2_jazzy_docker_template
```

Crazyswarm2 is pinned through `.env`:

```bash
CRAZYSWARM2_REF=fcf51e27fe675bbdb564a40f4ca1ee4336c062ea
```

## 2. Configure OptiTrack

Edit:

```bash
nano config/motion_capture.yaml
```

Set the Motive IP address:

```yaml
/motion_capture_tracking:
  ros__parameters:
    type: "optitrack"
    hostname: "192.168.0.4"
```

Rigid body names in Motive must match the robot names in `config/crazyflies.yaml`, for example:

```text
cf_1
cf_2
cf_4
cf_7
```

## 3. Configure the drones

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
    initial_position: [0.0, 0.0, 0.0]
    type: bolt_3in_2s

  cf_2:
    enabled: true
    uri: radio://0/80/2M/E7E7E7EA02
    initial_position: [0.0, 0.0, 0.0]
    type: bolt_3in_2s
```

The current OptiTrack flight configuration uses:

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

## 4. Build the Docker image

```bash
source .env

docker build \
  --build-arg CRAZYSWARM2_REF="$CRAZYSWARM2_REF" \
  -t crazyswarm2:jazzy-pinned \
  -f docker/Dockerfile \
  .
```

## 5. Start the stack

Make the helper scripts executable once:

```bash
chmod +x docker/start.sh docker/shell.sh
```

Start Crazyswarm2, OptiTrack tracking, and RViz2:

```bash
./docker/start.sh
```

In RViz2 set:

```text
Fixed Frame = mocap
```

You should see the OptiTrack frames such as:

```text
cf_1_mocap
cf_2_mocap
```

## 6. Open a ROS shell

From another host terminal:

```bash
./docker/shell.sh
```

All ROS commands below are executed inside this shell.

Check the active nodes:

```bash
ros2 node list
```

Check OptiTrack:

```bash
ros2 topic echo /poses --once
```

Check a drone pose estimate:

```bash
ros2 topic echo /cf_1/pose --once
```

## 7. Basic takeoff and landing

Arm all enabled drones:

```bash
ros2 service call /all/arm \
  crazyflie_interfaces/srv/Arm \
  "{arm: true}"
```

Take off to 30 cm:

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

## 8. Run the upstream Crazyswarm2 example

With the server already running:

```bash
ros2 run crazyflie_examples hello_world
```

For a first test, enable only one drone in `config/crazyflies.yaml`.

## 9. Custom waypoint experiments

The `scripts/` directory is mounted directly into the container, so scripts can be edited on the host and executed without rebuilding the image.

### Single drone with `goTo()`

```bash
python3 /workspace/scripts/waypoints_goto.py
```

### Multiple drones with `goTo()`

```bash
python3 /workspace/scripts/waypoints_multi_drones.py
```

### Single drone with `cmdFullState()`

```bash
python3 /workspace/scripts/waypoints_full_state.py
```

### Multiple drones with `cmdFullState()`

```bash
python3 /workspace/scripts/waypoints_full_state_multi.py
```

The `cmdFullState()` scripts generate smooth seventh-order polynomial segments and continuously stream position, velocity, and acceleration setpoints at 30 Hz.

For the transition back to high-level landing, the scripts call `notifySetpointsStop()` and immediately send the landing command without inserting a delay between them.

## 10. Emergency stop

If the radio connection is still available:

```bash
ros2 service call /all/emergency \
  std_srvs/srv/Empty \
  "{}"
```

This cuts motor power immediately. It is not a controlled landing.

The custom waypoint scripts also use `Ctrl+C` as an emergency stop while they are running.

A firmware-side radio-loss watchdog is **not yet integrated** into this template.

## 11. Typical workflow

Terminal 1:

```bash
cd ~/docker_projects/crazyswarm2_jazzy_docker_template
./docker/start.sh
```

Terminal 2:

```bash
cd ~/docker_projects/crazyswarm2_jazzy_docker_template
./docker/shell.sh
```

Then execute the desired experiment, for example:

```bash
python3 /workspace/scripts/waypoints_full_state_multi.py
```

## Stop

Stop the launch with `Ctrl+C` in the terminal running `docker/start.sh`.