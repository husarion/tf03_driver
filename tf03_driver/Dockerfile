FROM ros:noetic-ros-core
SHELL ["/bin/bash", "-c"]

# install dependencies
RUN apt update && apt install -y \
    python3-pip \
    libserial-dev \
    can-utils \
    net-tools
RUN apt upgrade -y

# create ros1_ws, copy and build package
RUN mkdir -p /ros1_ws/src
COPY ./tf03_driver /ros1_ws/src/tf03_driver

WORKDIR /ros1_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin_make

# clear ubuntu packages
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
RUN bash -c "chmod +x /ros_entrypoint.sh"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]