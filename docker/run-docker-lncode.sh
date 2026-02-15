#!/usr/bin/env bash
REPO=$(cd ..; pwd)
xhost + local:docker
docker run -it --rm \
  --network=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /data:/data \
  -v $REPO/ogm:/home/ogm-cbf/ogm \
  -v $REPO/ros2_ws/src/ogm_cbf_kinematic_sim:/home/ogm-cbf/ogm_cbf_kinematic_sim \
  amm-ogm-cbf-eevee
