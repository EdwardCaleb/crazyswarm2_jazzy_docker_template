#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

PROJECT_NAME="$(basename "$PROJECT_ROOT")"
CONTAINER_NAME="${PROJECT_NAME}_dev"

echo "Project:   $PROJECT_NAME"
echo "Container: $CONTAINER_NAME"
echo
echo "Starting Docker environment..."
echo "Ctrl+C here will stop the container and all attached sessions."
echo

# Allow GUI applications inside Docker to use the host X server.
xhost +SI:localuser:root >/dev/null

docker compose run --rm \
  --name "$CONTAINER_NAME" \
  crazyswarm2 \
  bash -lc '
    echo "Container ready."
    echo "Use ./docker/ros.sh --real or --sim from another terminal."
    echo "Use ./docker/shell.sh for additional shells."
    echo

    exec sleep infinity
  '