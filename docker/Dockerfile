# ============================================================
# 🐳 Husky + Velodyne 3D LiDAR Simulation (ROS 2 Foxy)
# Ubuntu 20.04 base image with Gazebo GUI and VineSLAM deps
# ============================================================

FROM ubuntu:20.04

# ==============================================
# Generic setup
# ==============================================
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \
    git vim wget curl gnupg2 lsb-release software-properties-common

RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ==============================================
# ROS 2 Foxy installation
# ==============================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y \
    ros-foxy-desktop \
    python3-colcon-common-extensions \
    python3-rosdep

RUN rosdep init && rosdep update

# ==============================================
# Gazebo + Husky + Velodyne packages
# ==============================================
RUN apt install -y \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-gazebo-ros2-control \
    ros-foxy-husky-gazebo \
    ros-foxy-husky-simulator \
    ros-foxy-husky-viz \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-velodyne \
    ros-foxy-velodyne-msgs \
    ros-foxy-velodyne-description \
    ros-foxy-velodyne-gazebo-plugins

# ==============================================
# GUI / Headless support
# ==============================================
RUN apt install -y \
    x11-apps mesa-utils libgl1-mesa-glx xvfb

# ==============================================
# VineSLAM dependencies
# ==============================================
RUN apt-get install -y \
    libopencv-dev \
    libboost-dev \
    libasio-dev \
    ros-foxy-diagnostic-updater \
    ros-foxy-vision-msgs \
    ros-foxy-ublox-msgs

# ==============================================
# Workspace setup
# ==============================================
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

# Example dependency (optional: VineSLAM or Ublox)
RUN git clone https://github.com/KumarRobotics/ublox -b foxy-devel

# ==============================================
# (Optional) Add custom Husky + Velodyne URDF
# ==============================================
# Copy your combined robot model if available
# COPY husky_with_velodyne.xacro /root/ros2_ws/src/husky_description/urdf/

# ==============================================
# Build workspace
# ==============================================
WORKDIR /root/ros2_ws
RUN . /opt/ros/foxy/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ==============================================
# Environment setup
# ==============================================
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# ==============================================
# Cleanup to reduce image size
# ==============================================
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# ==============================================
# Default command (interactive shell)
# ==============================================
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
