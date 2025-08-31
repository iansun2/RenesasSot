#!/bin/bash

export WAYLAND_DISPLAY=wayland-0

docker kill sot
docker rm sot

DIR="$( cd "$( dirname "$0" )" && pwd )"
echo "DIR: ${DIR}"

docker run -it -d --name sot \
    --privileged \
    --net=host \
    -e XDG_RUNTIME_DIR=/tmp \
	-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -v $XDG_RUNTIME_DIR/wayland-1:/tmp/wayland-1 \
	-e QT_QPA_PLATFORM=wayland \
    -v ${DIR}/workspace:/root/workspace \
    -v ${DIR}/ssh/:/root/.ssh \
    renesas_sot

    # -v ${DIR}/user_data/pip-packages:/usr/local/lib/python3.10/dist-packages \
    # -v ${DIR}/user_data/local:/root/.local \
    # -v $XDG_RUNTIME_DIR/wayland-0:/tmp/wayland-0 \
    # -v $XDG_RUNTIME_DIR/wayland-1:/tmp/wayland-1 \
