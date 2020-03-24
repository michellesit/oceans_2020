#!/usr/bin/env bash

set -e

export DEBIAN_FRONTEND=noninteractive

# apt-get install software-properties-common
# add-apt-repository ppa:deadsnakes/ppa

apt-get update
apt-get install --yes vim

# apt-get install --yes python3.7
# update-alternatives --install /usr/bin/python python /usr/bin/python3 1

apt-get install --yes python-pip \
	python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose \
	python-catkin-tools

pip install numpy-stl
