#!/bin/bash
echo "Attempting to bring down CAN interfaces"
./shutdownCAN.sh
echo "Attempting to bring up CAN interfaces"
./setupCAN.sh
. devel/setup.bash
roslaunch tfr_launch robot.launch
