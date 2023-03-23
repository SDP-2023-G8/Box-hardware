#!/bin/sh
/usr/local/bin/mjpg_streamer -i "input_uvc.so -r 640x480 -f 24" -o "output_http.so -l 0.0.0.0 -p 8090 -w /usr/local/share/mjpg-streamer/www"