#!/bin/sh
# assume we are in the NasaRmc2019 directory.
echo "Begin installing lpms_imu_driver..."
cd src/lpms_imu_driver/
tar xvzf LpSensor-1.3.5-Linux-x86-64.tar.gz
sudo apt-get install libbluetooth-dev
sudo dpkg -i LpSensor-1.3.5-Linux-x86-64/liblpsensor-1.3.5-Linux.deb
dpkg -L liblpsensor
rm -rf LpSensor-1.3.5-Linux-x86-64
cd ../ # src
cd ../ # NasaRmc2019
echo "Finished installing lpms_imu_driver."
