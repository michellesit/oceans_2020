#!/usr/bin/env bash

set -e

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install --yes python-catkin-tools

cd /home$USER/
git clone https://github.com/michellesit/oceans_2020.git
cd oceans_2020
catkin init

# Install uuv simulator
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y --skip-keys "gazebo gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control gazebo_ros_pkgs"
apt-get install --yes ros-kinetic-uuv-simulator

# Install rexrov2 simulator
cd ~/src
git clone https://github.com/uuvsimulator/rexrov2.git

# Install uuv simulator environment
git clone https://github.com/uuvsimulator/uuv_simulation_evaluation.git

catkin build

