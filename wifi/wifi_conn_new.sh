#!/bin/bash

if [ "$1" == "" ]; then
	echo "no SSID";
	exit 0;
fi


if [ "$2" == "" ]; then
	echo "no PASSWORD";
	exit 0;
fi



echo "SSID:" "$1"
echo "PASSWORD:" "$2"

wpa_passphrase "$1" "$2" > /etc/wpa_supplicant.conf 
wpa_supplicant -Dwext -iwlan0 -c/etc/wpa_supplicant.conf
# wpa_supplicant -B -Dwext -iwlan0 -c/etc/wpa_supplicant.conf

# sleep 10;
# udhcpc -i wlan0

# echo "Get IP:"
# ifconfig wlan0 | grep inet | awk -F " " {'print $2'}

