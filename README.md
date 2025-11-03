# docker building
- check you .env for correct ROS_DISTRO, LIDAR_IP and HOST_IP envs
- execute get-env.sh
- start docker compose with *docker compose up --build*

# building driver + sdk
- enter container with *docker execute -it (container name) bash*
- execute build-sdk.sh in the container 
- activate environment with *source ws/install/setup.bash*. It includes env from all of the packages (livox sdk, livox ros, kitti writer)
