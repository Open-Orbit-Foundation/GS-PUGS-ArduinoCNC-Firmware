#!/bin/bash
# stream-webcam.sh

ffmpeg -f v4l2 -input_format mjpeg -framerate 30 -video_size 640x480 -i /dev/video0 \
  -pix_fmt yuv420p \
  -c:v libx264 \
  -preset ultrafast \
  -tune zerolatency \
  -profile:v baseline \
  -b:v 1500000 \
  -maxrate 1500000 \
  -bufsize 50000 \
  -g 30 \
  -keyint_min 30 \
  -sc_threshold 0 \
  -x264-params "nal-hrd=cbr:force-cfr=1:ref=1:bframes=0" \
  -fflags nobuffer+flush_packets \
  -flags low_delay \
  -max_delay 0 \
  -muxdelay 0 \
  -f mpegts \
  -muxrate 1600000 \
  -listen 1 \
  http://0.0.0.0:8080
