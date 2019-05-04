#!/bin/sh
#sudo ip link set up can1
sudo ifconfig can0 down
sudo ifconfig can1 down
#sudo ip link set can1 type can bitrate 125000
sudo modprobe -r mttcan
sudo modprobe -r can_raw
sudo modprobe -r can

