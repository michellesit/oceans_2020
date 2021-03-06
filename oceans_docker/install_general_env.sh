#!/usr/bin/env bash

set -e

export DEBIAN_FRONTEND=noninteractive

# apt-get install software-properties-common
# add-apt-repository ppa:deadsnakes/ppa

apt-get update
apt-get install --yes vim

## Switches default python to python3
# apt-get install --yes python3.7
# update-alternatives --install /usr/bin/python python /usr/bin/python3 1

apt-get install --yes python-pip \
	python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose \
	python-catkin-tools

pip install numpy==1.13.0
pip install numpy-stl Shapely

## Optional python3 packages:
apt-get install --yes python3-numpy python3-scipy python3-matplotlib python3-pip
pip2 install numpy==1.13.0
pip3 install numpy-stl
