# docker building
- change lines in .devcontainer/devcontainer.json and Dockerfile that marked with comments;
- make sure you have livox_ros_driver2 and Livox-SDK2 packages in src/;
- install Remote Development Extension for your VSCode;
- execute command Dev Containers: Reopen in Container in Commands Pallete.

# building driver + sdk
- build and install Livox-SDK2 by following Livox-SDK2/README.md;
- change config params in src/livox_ros_driver2/config/*your lidar*.json (define host and lidar ips);
- execute "source /opt/ros/humble/setup.sh" to activate ros2 venv;
- execute ./build.sh from src/livox_ros_driver2;
- execute "source install/setup.sh" to activate build venv (correct venvs' activating order is humble venv -> build venv in single terminal);
- launch rviz2 using livox driver with "ros2 launch livox_ros_driver2 rviz_*your lidar*_launch.py";