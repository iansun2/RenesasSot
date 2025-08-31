#!/bin/bash

export WAYLAND_DISPLAY=wayland-1
docker start sot

docker run -it --rm -d --name redis \
    --net=host \
    redis:7.4.1

docker run -it --rm -d --name ntp \
    -p  123:123/udp \
    --env=NTP_SERVERS="127.127.1.1" \
    dockurr/chrony
