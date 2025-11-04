ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    autoconf libtool \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-xacro && \
    : "remove cache" && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/ethercat_ws
RUN git clone https://gitlab.com/etherlab.org/ethercat.git && \
    cd ethercat && \
    git checkout stable-1.5 && \
    ./bootstrap && \
    ./configure --prefix=/usr/local/etherlab --disable-kernel && \
    make && make install
    
# build ros2_control framework
# and ethercat plugins
WORKDIR /home/ros2_ws
RUN \
  # set -x for debug output
  set -x && \
  apt-get update -y -qq && \
  : "install ros2_control from src" && \
  mkdir src && \
  vcs import --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.rolling-on-$ROS_DISTRO.repos src && \
  git clone https://github.com/christophfroehlich/ethercat_driver_ros2.git src/ethercat_driver_ros2 -b AIT && \
  rosdep update --rosdistro=$ROS_DISTRO && \
  . /opt/ros/${ROS_DISTRO}/setup.sh && \
  rosdep install -riy --from-paths src && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --symlink-install \
  --packages-skip \
    ackermann_steering_controller admittance_controller bicycle_steering_controller chained_filter_controller diff_drive_controller effort_controllers force_torque_sensor_broadcaster forward_command_controller gpio_controllers gps_sensor_broadcaster imu_sensor_broadcaster joint_state_broadcaster joint_trajectory_controller mecanum_drive_controller motion_primitives_controllers omni_wheel_drive_controller parallel_gripper_controller pid_controller position_controllers range_sensor_broadcaster ros2_controllers ros2_controllers_test_nodes rqt_joint_trajectory_controller steering_controllers_library tricycle_controller tricycle_steering_controller velocity_controllers \
    joint_state_topic_hardware_interface rqt_controller_manager kinematics_interface kinematics_interface_kdl && \
  : "remove cache" && \
  rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY Dockerfile/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]