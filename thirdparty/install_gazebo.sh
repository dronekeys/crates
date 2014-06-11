#!/bin/bash

# Number of compilation threads (reduce if there are problems with vm calculations)
NT=2

# Save the base directory
BASEDIR=${PWD}

# Install sdformat
cd $BASEDIR
if [[ ! -d $BASEDIR/sdformat ]]; then
	hg clone https://bitbucket.org/osrf/sdformat
fi
cd sdformat
hg up sdformat_2.0.0
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ../
make -j$NT
sudo make install

# Install gazebo
cd $BASEDIR
if [[ ! -d $BASEDIR/gazebo ]]; then
	hg clone https://bitbucket.org/osrf/gazebo
fi
cd gazebo
hg up gazebo3_3.0.0
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_TESTS_COMPILATION:BOOL=False -DCMAKE_BUILD_TYPE=Release ../
make -j$NT
sudo make install

# Create a home directory or gazebo wont start
mkdir -p ~/.gazebo/models