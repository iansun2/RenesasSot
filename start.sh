#!/bin/bash

export WAYLAND_DISPLAY=wayland-1
docker start sot
docker start redis