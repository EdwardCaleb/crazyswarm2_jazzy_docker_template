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
    echo "  ./docker/start.sh --real"
    echo "  ./docker/start.sh --sim"
    exit 1
    ;;
esac

echo "Project:   $PROJECT_NAME"
echo "Container: $CONTAINER_NAME"
echo "Mode:      $MODE"

# Allow GUI applications inside Docker to use the host X server.
xhost +SI:localuser:root >/dev/null

docker compose run --rm \
  --name "$CONTAINER_NAME" \
  crazyswarm2 \
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