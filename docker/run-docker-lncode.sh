#!/usr/bin/env bash
docker run -it --rm \
  --network=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /data:/data \
  -v ../ogm:/home/ogm-cbf/ogm \
  -v ../ros2_ws/src/ogm_cbf_kinematic_sim:/home/ogm-cbf/ogm_cbf_kinematic_sim \
  amm-ogm-cbf
