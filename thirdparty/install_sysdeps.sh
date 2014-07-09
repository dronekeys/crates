#!/bin/bash

# Add oracle java support
sudo apt-get install python-software-properties
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | sudo /usr/bin/debconf-set-selections

# Basic libraries
sudo apt-get -qq -y install gdal-bin libgdal-dev python-gdal gradle build-essential \
    doxygen libtinyxml2-dev libkml-dev libboost-all-dev cmake mercurial pkg-config \
 	libprotoc-dev libprotobuf-dev protobuf-compiler libqt4-dev libtar-dev \
 	libcurl4-openssl-dev libcegui-mk2-dev libopenal-dev libtbb-dev \
 	libswscale-dev libavformat-dev libavcodec-dev libogre-1.8-dev libgts-dev libltdl3-dev \
 	playerc++ libxml2-dev libfreeimage-dev freeglut3-dev libgeographiclib-dev oracle-java8-installer

