ARG ROS_DISTRO=huba_buba
ARG USERNAME=huba_buba

FROM ros:${ROS_DISTRO} 

ARG USERNAME=${USERNAME}
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip 
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
WORKDIR /home
RUN mkdir -p src/ros2_pkg
COPY ../ws/src/livox_ros_driver2/package_ROS2.xml src/ros2_pkg/package.xml
RUN sudo rosdep update
RUN sudo rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN sudo apt update && sudo apt install ros-${ROS_DISTRO}-rviz2 cmake -y
RUN sudo apt install -y ros-humble-image-view ros-humble-pcl-ros ros-humble-pcl-conversions
RUN rm -rdf src/
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["tail", "-f", "/dev/null"]