#!/bin/bash
export $(grep -v '^#' .env | xargs)
set -e  
cd /home/$USERNAME/ws/src/Livox-SDK2
if [[ ! -d ../../build/livox_sdk2 ]]; then
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install 
fi

cd /home/$USERNAME/ws/src/livox_ros_driver2
if [[ ! -d ../../build/livox_ros_driver2 ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    ./build.sh $ROS_DISTRO
fi

cd /home/$USERNAME/ws
if [[ ! -d ../../build/kitti_writer ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --packages-select kitti_writer
fi

chown 