#!/bin/bash
./set_ip.sh
wpa_supplicant -Dwext -iwlan0 -c/etc/wpa_supplicant.conf

# sleep 10;
# udhcpc -i wlan0

# echo "Get IP:"
# ifconfig wlan0 | grep inet | awk -F " " {'print $2'}

