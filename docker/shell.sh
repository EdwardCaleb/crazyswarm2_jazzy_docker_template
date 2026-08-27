#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_NAME="$(basename "$PROJECT_ROOT")"
CONTAINER_NAME="${PROJECT_NAME}_dev"

docker exec -it "$CONTAINER_NAME" \
  bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/crazyswarm2_ws/install/setup.bash
    exec bash
  '