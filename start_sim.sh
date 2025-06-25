#!/usr/bin/env bash
set -e

COMPOSE_FILE="docker/docker-compose.yml"
CONTAINER_NAME=ros2_container

cd "$( dirname "${BASH_SOURCE[0]}" )"

if docker ps -a --format '{{.Names}}' | grep -xq "${CONTAINER_NAME}"; then
  echo "[INFO] Found existing container ${CONTAINER_NAME}, removing it..."
  docker rm -f "${CONTAINER_NAME}"
fi

docker compose -f "${COMPOSE_FILE}" up --force-recreate -d

cleanup() {
  echo
  echo "[INFO] Stopping and removing container ${CONTAINER_NAME}..."
  docker compose -f "${COMPOSE_FILE}" down
}
trap cleanup EXIT

docker exec -it "${CONTAINER_NAME}" bash -lc "\
  source /home/robot/deburring_robot_ws/install/setup.bash && \
  ros2 launch deburring_robot_bringup simulated_robot.launch.py"