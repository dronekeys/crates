#!/bin/bash

# Save the base directory
BASEDIR=${PWD}

# Install sdformat
cd $BASEDIR
if [[ ! -d $BASEDIR/sdformat ]]; then
	hg clone https://bitbucket.org/osrf/sdformat
fi
cd sdformat
hg up sdf_2.0
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug ../
make -j4
sudo make install

# Install gazebo
cd $BASEDIR
if [[ ! -d $BASEDIR/gazebo ]]; then
	hg clone https://bitbucket.org/osrf/gazebo
fi
cd gazebo
hg up gazebo_3.0
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_TESTS_COMPILATION:BOOL=False -DCMAKE_BUILD_TYPE=Debug ../
make -j4
sudo make install

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