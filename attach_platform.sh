#!/bin/bash

# docker pull arm64v8/ros:humble-perception


docker exec -w /root/workspace/platform_ws -it sot bash -c " \
    source /opt/ros/humble/setup.bash; \
    bash"

#     . /ros_entrypoint.sh; \