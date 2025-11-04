ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    autoconf libtool \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-pose-broadcaster ros-${ROS_DISTRO}-xacro && \
    : "remove cache" && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/ethercat_ws
RUN git clone https://gitlab.com/etherlab.org/ethercat.git && \
    cd ethercat && \
    git checkout stable-1.5 && \
    ./bootstrap && \
    ./configure --prefix=/usr/local/etherlab --disable-kernel && \
    make && make install

WORKDIR /home/ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git src/ethercat_driver_ros2 && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install && \
    : "remove cache" && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY Dockerfile/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]