# Crazyswarm2 + ROS 2 Jazzy Docker Template

Minimal setup for running **Crazyswarm2**, **ROS 2 Jazzy**, **OptiTrack/Motive**, **Crazyradio PA**, **RViz2**, and Crazyflie/Bolt vehicles inside Docker on Ubuntu 24.04.

## Requirements

- Ubuntu 24.04
- Docker Engine + Docker Compose
- Crazyradio PA
- OptiTrack Motive on the same network as the host
- X11 desktop session for RViz

## 1. Clone

```bash
git clone <YOUR_REPOSITORY_URL>
cd crazyswarm2_jazzy_docker_template
```

The project pins Crazyswarm2 through `.env`:

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

Rigid body names in Motive must match the robot names in the Crazyflie configuration, for example:

```text
cf_1
cf_2
```

## 3. Configure the Crazyflies

Edit:

```bash
nano config/crazyflies_link_test.yaml
```

Set your robot URIs and enable the vehicles you want to use:

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

For OptiTrack-based flight, the current setup uses:

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

## 4. Build the image

```bash
source .env

docker build \
  --build-arg CRAZYSWARM2_REF="$CRAZYSWARM2_REF" \
  -t crazyswarm2:jazzy-pinned \
  -f docker/Dockerfile \
  .
```

## 5. Allow RViz to open from Docker

On the host:

```bash
xhost +SI:localuser:root
```

## 6. Start the container

```bash
docker compose run --rm \
  --name crazyswarm2_dev \
  crazyswarm2 \
  bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/crazyswarm2_ws/install/setup.bash
```

Optional Crazyradio check:

```bash
lsusb | grep 1915:7777
```

## 7. Launch Crazyswarm2

Inside the container:

```bash
ros2 launch crazyflie launch.py \
  backend:=cflib \
  mocap:=True \
  teleop:=False \
  rviz:=True \
  crazyflies_yaml_file:=/workspace/config/crazyflies_link_test.yaml \
  motion_capture_yaml_file:=/workspace/config/motion_capture.yaml
```

In RViz set:

```text
Fixed Frame = mocap
```

## 8. Open a second shell

From another host terminal:

```bash
docker exec -it crazyswarm2_dev bash
```

Inside:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/crazyswarm2_ws/install/setup.bash
```

Check OptiTrack data:

```bash
ros2 topic echo /poses --once
```

## 9. Arm, take off, and land

Arm all enabled vehicles:

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

## 10. Run the Crazyswarm2 example

With the server still running:

```bash
ros2 run crazyflie_examples hello_world
```

For a first test, enable only one vehicle in `crazyflies_link_test.yaml`.

## 11. Run the custom waypoint script

```bash
python3 /workspace/scripts/waypoints_goto.py
```

Edit it directly on the host:

```bash
nano scripts/waypoints_goto.py
```

Changes under `config/` and `scripts/` do not require rebuilding the Docker image.

## Emergency stop

If communication is still available:

```bash
ros2 service call /all/emergency \
  std_srvs/srv/Empty \
  "{}"
```

This immediately cuts motor power. It is not a controlled landing.

## Stop

Stop the launch with `Ctrl+C`, then exit the container:

```bash
exit
```

Revoke X11 access if desired:

```bash
xhost -SI:localuser:root
```
