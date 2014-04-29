#!/bin/bash

# Add repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update

# Install packages
sudo apt-get install libdc1394-22-dev libdc1394-utils
sudo apt-get install libpcl-1.7-all-dev
sudo apt-get install ros-indigo-roscpp

# Prepare ROS
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Checkout code