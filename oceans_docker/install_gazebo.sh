#!/usr/bin/env bash

##Borrowed from Shengye's docker registry

set -e

export DEBIAN_FRONTEND=noninteractive

apt-get purge --yes ros-kinetic-gazebo* libgazebo* gazebo*
apt-mark showauto | xargs apt-get install --yes

echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

apt-get update
apt-get install --yes ros-kinetic-gazebo9-*
apt-get install --yes ros-kinetic-ros-base ros-kinetic-catkin rviz \
  ros-kinetic-controller-manager ros-kinetic-joint-state-controller \
  ros-kinetic-joint-trajectory-controller ros-kinetic-rqt \
  ros-kinetic-rqt-controller-manager \
  ros-kinetic-rqt-joint-trajectory-controller ros-kinetic-ros-control \
  ros-kinetic-rqt-gui ros-kinetic-rqt-plot ros-kinetic-rqt-graph \
  ros-kinetic-rqt-rviz ros-kinetic-rqt-tf-tree \
  ros-kinetic-gazebo9-ros ros-kinetic-kdl-conversions \
  ros-kinetic-kdl-parser ros-kinetic-forward-command-controller \
  ros-kinetic-tf-conversions ros-kinetic-xacro \
  ros-kinetic-joint-state-publisher ros-kinetic-robot-state-publisher \
  ros-kinetic-ros-control ros-kinetic-ros-controllers

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
