#!/bin/bash
ip a add 192.168.186.3/24 dev usb0
ip link set dev usb0 up
