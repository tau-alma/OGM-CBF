#!/usr/bin/env bash
docker run -it --rm \
  --network=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /data:/data \
  amm-ogm-cbf
