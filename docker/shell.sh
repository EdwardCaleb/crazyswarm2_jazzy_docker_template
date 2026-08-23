#!/usr/bin/env bash

set -euo pipefail

docker exec -it crazyswarm2_dev \
  bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/crazyswarm2_ws/install/setup.bash
    exec bash
  '
