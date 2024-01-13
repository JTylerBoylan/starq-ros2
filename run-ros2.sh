#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Build the Docker image
docker build -t starq-ros2:latest "${SCRIPT_DIR}"

# Start the Docker container
docker run -it \
    --rm \
    --net host \
    -v "/${SCRIPT_DIR}:/ros2_ws/src/" \
    starq-ros2:latest \
    bash