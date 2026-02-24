#!/bin/bash
xhost +local:root

docker run -it --rm \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=7 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  jazzy-desktop-cyclone
