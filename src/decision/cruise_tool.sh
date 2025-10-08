#! /bin/bash

# enable kernel module: gs_usb
sudo modprobe gs_usb

# bring up can interface
sudo ip link set can0 up type can bitrate 500000

# install can utils
sudo apt install -y can-utils

gnome-terminal -x bash -c "roslaunch ranger_bringup ranger_mini_v2.launch"&
# sleep 1
# gnome-terminal -x bash -c "roslaunch cruise_nav tool.launch "
wait
exit 0
