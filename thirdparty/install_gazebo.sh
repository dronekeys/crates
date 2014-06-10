#!/bin/bash

# Add to apt
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'

# Add the key
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update the libraries
sudo apt-get update

# Install base libraries
sudo apt-get -yinstall libgazebo-dev

# Save the base directory
BASEDIR=${PWD}

# Install gazebo models
cd $BASEDIR
if [[ ! -d $BASEDIR/gazebo_models ]]; then
	hg clone https://bitbucket.org/osrf/gazebo_models
fi
cd gazebo_models
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug ../
make -j4
sudo make install

# Create a home directory or gazebo wont start
mkdir -p ~/.gazebo

# Create a link to the recently-downloaded models
ln -s /usr/local/models ~/.gazebo/models