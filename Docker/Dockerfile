FROM osrf/ros:melodic-desktop-full

ARG DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO melodic

ENV SHELL=/bin/bash \
    USER=arg \
    UID=1000

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:111111" | chpasswd

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    make \
    git \
    vim \
    wget \
    sudo \
    lsb-release \
    build-essential \
    net-tools

RUN cd ${HOME} && git clone https://github.com/lcm-proj/lcm \
    && cd lcm \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

RUN cd ${HOME} && git clone http://github.com/ARG-NCTU/procman \
    && cd procman \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    ros-$ROS_DISTRO-pointgrey-camera-description \
    ros-$ROS_DISTRO-robot-upstart \
    ros-$ROS_DISTRO-nmea-navsat-driver \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-rosserial-server \
    ros-$ROS_DISTRO-velodyne-description \
    ros-$ROS_DISTRO-interactive-marker-twist-server \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-nmea-comms \
    ros-$ROS_DISTRO-sick-tim \
    ros-$ROS_DISTRO-map-server \
    ros-$ROS_DISTRO-pointgrey-camera-driver \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-lms1xx \
    ros-$ROS_DISTRO-gmapping \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-velodyne-pointcloud \
    ros-$ROS_DISTRO-urg-node \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-rosdoc-lite \
    ros-$ROS_DISTRO-rosserial-python \
    ros-$ROS_DISTRO-amcl \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-axis-camera \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    python-scipy \
    python3-scipy \
    ros-$ROS_DISTRO-cartographer \
    && rosdep update \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
