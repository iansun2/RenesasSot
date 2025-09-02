#!/bin/bash

# docker pull arm64v8/ros:humble-perception


docker exec -w /root/workspace/arm_ws2 -it sot bash -c " \
    source /opt/ros/humble/setup.bash; \
    bash"

#     . /ros_entrypoint.sh; \