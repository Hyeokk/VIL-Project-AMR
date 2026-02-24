#!/bin/bash
xhost +local:root

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

docker run -it --rm \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=64 \
  -e CYCLONEDDS_URI=file:///tmp/cyclonedds.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $SCRIPT_DIR/cyclonedds.xml:/tmp/cyclonedds.xml:ro \
  -v $SCRIPT_DIR/monitor.rviz:/tmp/monitor.rviz:ro \
  jazzy-desktop-cyclone \
  bash -c "source /opt/ros/jazzy/setup.bash && rviz2 -d /tmp/monitor.rviz"