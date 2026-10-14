#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

PROJECT_NAME="$(basename "$PROJECT_ROOT")"
CONTAINER_NAME="${PROJECT_NAME}_dev"

MODE="${1:-}"

case "$MODE" in
  --real)
    BACKEND="cflib"
    MOCAP="True"
    ;;

  --sim)
    BACKEND="sim"
    MOCAP="False"
    ;;

  *)
    echo "Usage:"
    echo "  ./docker/ros.sh --real"
    echo "  ./docker/ros.sh --sim"
    exit 1
    ;;
esac


# Make sure the project container is running.
if ! docker ps \
  --format '{{.Names}}' \
  | grep -qx "$CONTAINER_NAME"; then

  echo "ERROR: container '$CONTAINER_NAME' is not running."
  echo
  echo "Start it first with:"
  echo "  ./docker/start.sh"
  exit 1
fi


echo "Container: $CONTAINER_NAME"
echo "Mode:      $MODE"
echo "Backend:   $BACKEND"
echo "Mocap:     $MOCAP"
echo


docker exec -it "$CONTAINER_NAME" \
  bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /opt/crazyswarm2_ws/install/setup.bash

    exec ros2 launch crazyflie launch.py \
      backend:=${BACKEND} \
      mocap:=${MOCAP} \
      teleop:=False \
      rviz:=True \
      crazyflies_yaml_file:=/workspace/config/crazyflies.yaml \
      motion_capture_yaml_file:=/workspace/config/motion_capture.yaml
  "