#!/bin/bash
export $(grep -v '^#' .env | xargs)
set -e  
cd /home/$USERNAME/ws/src/Livox-SDK2
if [[ ! -d build ]]; then
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install 
fi

cd /home/$USERNAME/ws/src/livox_ros_driver2
if [[ ! -d ../build/livox_ros_driver2 ]]; then
    source /opt/ros/$ROS_DISTRO/setup.sh
    ./build.sh $ROS_DISTRO
fi