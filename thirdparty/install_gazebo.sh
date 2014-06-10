#!/bin/bash

# Add to apt
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'

# Add the key
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update the libraries
sudo apt-get update

# Install base libraries
sudo apt-get -y install libgazebo-dev

# Create a home directory or gazebo wont start
mkdir -p ~/.gazebo