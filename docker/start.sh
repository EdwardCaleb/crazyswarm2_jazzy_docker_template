#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

PROJECT_NAME="$(basename "$PROJECT_ROOT")"
CONTAINER_NAME="${PROJECT_NAME}_dev"


cleanup() {
  echo
  echo "Stopping container: $CONTAINER_NAME"

  docker stop \
    --time 3 \
    "$CONTAINER_NAME" \
    >/dev/null 2>&1 || true

  echo "Container stopped."
}


# Always clean up when this script exits.
trap cleanup EXIT

# Ctrl+C
trap 'exit 130' INT

# kill / termination
trap 'exit 143' TERM


# Prevent accidentally starting a second container
# for the same project.
if docker ps -a \
  --format '{{.Names}}' \
  | grep -qx "$CONTAINER_NAME"; then

  echo "ERROR: container '$CONTAINER_NAME' already exists."
  echo
  echo "If it is no longer needed, remove it with:"
  echo "  docker rm -f $CONTAINER_NAME"

  exit 1
fi


# Allow GUI applications inside Docker to use the host X server.
xhost +SI:localuser:root >/dev/null


echo "Project:   $PROJECT_NAME"
echo "Container: $CONTAINER_NAME"
echo
echo "Starting Docker environment..."


# Start the container in the background.
docker compose run \
  -d \
  --rm \
  --name "$CONTAINER_NAME" \
  crazyswarm2 \
  sleep infinity \
  >/dev/null


echo
echo "Container ready."
echo
echo "Use:"
echo "  ./docker/ros.sh --real"
echo "  ./docker/ros.sh --sim"
echo "  ./docker/shell.sh"
echo
echo "Press Ctrl+C here to stop the complete environment."
echo


# Keep this script attached to the lifetime of the container.
docker wait "$CONTAINER_NAME" >/dev/null