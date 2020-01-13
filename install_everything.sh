#!/bin/bash
yes | sudo ./install_opencv_and_contrib.sh 2>&1 | tee ../opencv_log.txt; yes | sudo ./install_librealsense.sh 2>&1 | tee ../librealsense_log.txt; source devel/setup.bash; sudo ./build_all.sh 2>&1 | tee ../build_all_log.txt; sudo ./install_lpms_imu_driver.sh
