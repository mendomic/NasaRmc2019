#!/bin/sh
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000
sudo ip link set can1 type can bitrate 250000
sudo ip link set up can1
sudo ifconfig can1 up
sudo ifconfig can1 txqueuelen 1000
echo "Done"
#ifconfig
# 3/7/2019
#
# This set up both the can0 and can1 interfaces. We have Waveshare CAN transcievers connected to the bus on both can0 and can1.
# We can ONLY see packets arriving on can0.
# We run:
#     candump -t a can0
# And can see heartbeats print out from both motor controllers.
# We run:
#     cansend can0 123#abcdabcd
# And we can see this sent message in our running candump.
#
# Multiple times we ran:
#     sudo reboot
# Then logged back into the jetson over ssh, and ran:
#     ./working_CAN_setup_3_7_19_v1.sh
# And tested candump and cansend immediately afterwards and they work.
