FROM ubuntu:bionic
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=melodic

RUN apt update
RUN apt -y install gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update
RUN apt -y install python-catkin-tools python-pip python-rosdep python-wstool ros-melodic-catkin
RUN rosdep init
RUN rosdep update

RUN mkdir -p /catkin_ws/src
COPY . /catkin_ws/src/pilz_robots
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y
