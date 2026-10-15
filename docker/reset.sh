#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

PROJECT_NAME="$(basename "$PROJECT_ROOT")"
CONTAINER_NAME="${PROJECT_NAME}_dev"


# ------------------------------------------------------------
# Helper: run reset.py inside a container
# ------------------------------------------------------------

run_reset_in_container() {

  docker exec -it "$CONTAINER_NAME" \
    bash -lc '
      source /opt/ros/jazzy/setup.bash
      source /opt/crazyswarm2_ws/install/setup.bash

      exec python3 /workspace/scripts/reset.py "$@"
    ' bash "$@"
}


# ------------------------------------------------------------
# Case 1: project container is already running
# ------------------------------------------------------------

if docker ps \
    --format '{{.Names}}' \
    | grep -qx "$CONTAINER_NAME"; then

  # PowerSwitch accesses Crazyradio directly.
  #
  # Do not allow reset while crazyflie_server is running,
  # otherwise both processes may try to use the radio.
  if docker exec "$CONTAINER_NAME" \
      pgrep -f "crazyflie_server" \
      >/dev/null 2>&1; then

    echo "ERROR: Crazyswarm2 ROS server is still running."
    echo
    echo "Stop ./docker/ros.sh first with Ctrl+C,"
    echo "then run this reset command again."
    exit 1
  fi


  echo "Using running container:"
  echo "  $CONTAINER_NAME"
  echo

  run_reset_in_container "$@"

  exit 0
fi


# ------------------------------------------------------------
# Case 2: project container is not running
#
# Start a temporary container only for the reboot operation.
# ------------------------------------------------------------

echo "Project container is not running."
echo "Starting temporary reset container..."
echo


docker compose run --rm \
  crazyswarm2 \
  bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/crazyswarm2_ws/install/setup.bash

    exec python3 /workspace/scripts/reset.py "$@"
  ' bash "$@"