#!/usr/bin/env bash

HEIGHT=480
HZ=10

rm "/tmp/mir_concave_cbf_h.mp4"
cat /tmp/mir_concave_cbf_h_*.png | ffmpeg -framerate $HZ -f image2pipe -i - -filter:v "scale='ceil(iw/2)*2':'ceil(ih/2)*2'" -vcodec libx264 -r $HZ -pix_fmt yuv420p "/tmp/mir_concave_cbf_h.mp4"
