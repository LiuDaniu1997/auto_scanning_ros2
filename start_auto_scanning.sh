#!/usr/bin/env bash
set -e

CONTAINER="ros2_container"
SOURCE_CMD="source install/setup.bash"

sudo chown -R 1010:1010 ./data

sudo rm -rf ./data/merged_point_cloud/*

docker exec -it "${CONTAINER}" bash -lc "
${SOURCE_CMD} &&
ros2 launch auto_scanning auto_scanning.launch.py &&
ros2 launch deburring_robot_gazebo rotate_model.launch.py &&
ros2 launch auto_scanning auto_scanning.launch.py &&
ros2 run auto_scanning align_merged_clouds_node"

sudo chown -R $(whoami):$(whoami) ./data