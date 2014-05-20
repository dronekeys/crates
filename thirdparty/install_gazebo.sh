#!/bin/bash

# Save the base directory
BASEDIR=${PWD}

# Basic libraries
sudo apt-get install build-essential libtinyxml-dev libboost-all-dev cmake mercurial pkg-config \
 	libprotoc-dev libprotobuf-dev protobuf-compiler libqt4-dev libtar-dev \
 	libcurl4-openssl-dev libcegui-mk2-dev libopenal-dev libtbb-dev \
 	libswscale-dev libavformat-dev libavcodec-dev libogre-1.8-dev libgts-dev libltdl3-dev \
 	playerc++ libxml2-dev libfreeimage-dev freeglut3-dev

# Install sdformat
cd $BASEDIR
hg clone https://bitbucket.org/osrf/sdformat
cd sdformat
hg up sdf_2.0
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug ../
make -j4
sudo make install

# Install gazebo
cd $BASEDIR
hg clone https://bitbucket.org/osrf/gazebo
cd gazebo
hg up gazebo_3.0
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_TESTS_COMPILATION:BOOL=False -DCMAKE_BUILD_TYPE=Debug ../
make -j4
sudo make install

# Install gazebo models
cd $BASEDIR
hg clone https://bitbucket.org/osrf/gazebo_models
cd gazebo_models
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug ../
make -j4
sudo make install

# Post-installation configuration
ln -s ~/.gazebo/models /usr/share/models
sudo sh -c 'echo "/usr/local/lib/x86_64-linux-gnu" > /etc/ld.so.conf.d/gazebo.config' 
sudo ldconfig