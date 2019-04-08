#! /bin/bash
export ROS_HOSTNAME=192.168.0.162
export WIRINGPI_GPIOMEM=1
source ~/NasaRmc2019/devel/setup.bash
exec "$@"
