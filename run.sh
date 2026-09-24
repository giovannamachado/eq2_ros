#!/bin/bash

# Ensure the script is run with bash
if [ -z "$BASH_VERSION" ]; then
    exec bash "$0" "$@"
fi

# Prompt the user for the ROS_DOMAIN_ID
read -p "Type the ROS_DOMAIN_ID (a integer, except 102 until 214, and lower than 233): " INPUT_DOMAIN
DOMAIN=${INPUT_DOMAIN}

# Validate that the input is an integer and within the specified range
if ! [[ "$DOMAIN" =~ ^[0-9]+$ ]]; then
    echo "Error: The ROS_DOMAIN_ID must be an integer."
    exit 1
fi

if [ "$DOMAIN" -gt 102 ] && [ "$DOMAIN" -lt 214 ]; then
    echo "Error: The ROS_DOMAIN_ID can't be between 102 to 214."
    exit 1
fi

if [ "$DOMAIN" -ge 233 ]; then
    echo "Error: The ROS_DOMAIN_ID must be lower than 233."
    exit 1
fi

IMAGE_VERSION="1.2.4"

read -p "Type the number of your Team: " INPUT_TEAM
TEAM_NUMBER=${INPUT_TEAM}

xhost +local:docker

echo "Starting container with ROS_DOMAIN_ID=${DOMAIN}..."
docker run -d -it \
  --name kortex_humble_4 \
  --gpus all \
  --privileged \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=${DOMAIN} \
  --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
  --mount type=bind,source="${PWD}",target=/app/project_ws/src/eq${TEAM_NUMBER}_ros/ \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /dev/video0:/dev/video0 \
  --device /dev/dri:/dev/dri \
  --cap-add=sys_nice \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  --net host \
  --ipc host \
  --shm-size=1g \
  kortex_humble:${IMAGE_VERSION}