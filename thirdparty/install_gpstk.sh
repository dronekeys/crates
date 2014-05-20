#!/bin/bash

# Save the base directory
BASEDIR=${PWD}

# Download gpstk
if [[ ! -f $BASEDIR/gpstk-2.4.src.tar.gz ]]; then
	wget "http://downloads.sourceforge.net/project/gpstk/gpstk/2.4/gpstk-2.4.src.tar.gz"
fi

# Unzip the source
tar -xzf gpstk-2.4.src.tar.gz

# Apply patches
cd $BASEDIR
cp ./patches/CMakeLists.txt ./gpstk-2.4.src/dev/CMakeLists.txt
cp ./patches/cmake_uninstall.cmake.in ./gpstk-2.4.src/dev/cmake_uninstall.cmake.in

# Build the source
cd $BASEDIR
mkdir -p ./gpstk-2.4.src/dev/build
cd ./gpstk-2.4.src/dev/build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug ../
make -j4
sudo make install
