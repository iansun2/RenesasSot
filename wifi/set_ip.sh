#/bin/bash
ip a add 192.168.1.17/24 dev wlan0
echo "Set IP:"
ifconfig wlan0 | grep inet | awk -F " " {'print $2'}