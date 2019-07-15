#!/bin/bash
ros_version=""
version=$(lsb_release -rs)
echo $version

if [ $version == "16.04" ]; then
  ros_version="kinetic"
elif [ $version == "18.04" ]; then
  ros_version="melodic"
else
  echo "Unknown/unsupported distro"
  exit
fi
sudo apt-get update;
sudo apt-get install ros-$ros_version-desktop-full ros-$ros_version-controller-manager ros-$ros_version-move-base ros-$ros_version-hardware-interface ros-$ros_version-effort-controllers ros-$ros_version-joint-state-controller ros-$ros_version-joint-trajectory-controller libsfml-dev ros-$ros_version-gps-common ros-$ros_version-move-base ros-$ros_version-moveit ros-$ros_version-moveit-ros ros-$ros_version-cv-camera ros-$ros_version-vision-opencv ros-$ros_version-rosserial ros-$ros_version-rosserial-arduino ros-$ros_version-rqt ros-$ros_version-rqt-common-plugins ros-$ros_version-rqt-robot-plugins qtbase5-dev ros-$ros_version-ros-control ros-$ros_version-ros-controllers libi2c-dev libgtest-dev ros-$ros_version-robot-localization ros-$ros_version-openni-launch ros-$ros_version-ddynamic-reconfigure -y
