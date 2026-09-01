#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

PROJECT_NAME="$(basename "$PROJECT_ROOT")"
CONTAINER_NAME="${PROJECT_NAME}_dev"


# PowerSwitch opens the Crazyradio directly.
# Do not run it while crazyflie_server owns the radio.
if docker ps \
    --format '{{.Names}}' \
    | grep -qx "$CONTAINER_NAME"; then

    echo "ERROR: $CONTAINER_NAME is still running."
    echo
    echo "Stop ./docker/start.sh first, then run reset again."
    exit 1
fi


docker compose run --rm \
  crazyswarm2 \
  bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /opt/crazyswarm2_ws/install/setup.bash

    python3 /workspace/scripts/reset.py $*
  "
