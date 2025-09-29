FROM ubuntu:20.04

# Generic
# --------------------------------------------- #
RUN apt update
RUN apt-get install -y git
RUN apt-get install -y vim
RUN apt-get install -y wget

RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime
RUN DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y software-properties-common
# --------------------------------------------- #

# ROS installation
# --------------------------------------------- #
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt update && apt install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-desktop
RUN apt-get install -y python3-colcon-*
# --------------------------------------------- #

# VineSLAM dependencies installation
# --------------------------------------------- #
RUN apt-get install -y libopencv-dev
RUN apt-get install -y libboost-dev
RUN apt-get install -y libasio-dev

RUN apt-get install -y ros-foxy-diagnostic-updater
RUN apt-get install -y ros-foxy-vision-msgs
RUN apt-get install -y ros-foxy-ublox-msgs
# --------------------------------------------- #

# VineSLAM workspace preparation
# --------------------------------------------- #
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/KumarRobotics/ublox -b foxy-devel
WORKDIR /root/ros2_ws
# --------------------------------------------- #

# CUDA installation
# --------------------------------------------- #
#RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
#RUN mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
#RUN add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
#RUN apt-get update
#RUN apt-get install -y cuda
# --------------------------------------------- #

# ROS environment variables setting
# --------------------------------------------- #
COPY ./ros_entrypoint.sh /root/.
RUN chmod 777 /root/ros_entrypoint.sh
ENTRYPOINT ["/root/ros_entrypoint.sh"]
CMD ["bash"]
# --------------------------------------------- #
