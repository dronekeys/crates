Installation Instructions
=========================

Install Ubuntu 13.04 or 13.10

	Visit http://www.ubuntu.com to find out how to install Ubuntu on your laptop.
	
Add the ROS PPA to your Ubuntu Sources

	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu raring main" > /etc/apt/sources.list.d/ros-latest.list' 
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add - 
	
Add the Gazebo PPA to your Ubuntu Sources

	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu saucy main" > /etc/apt/sources.list.d/gazebo-latest.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	
Add the latest node.js PPS to your Ubuntu Sources

	sudo add-apt-repository ppa:chris-lea/node.js
	
Install system packages and remove those with known problems

	sudo apt-get update
	sudo apt-get autoremove nodejs
	sudo apt-get install gazebo-current python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential libjansson-dev nodejs npm libboost-dev
	
Initialise and update ROS

	sudo rosdep init 
	rosdep update 

Create and switch to a new ROS workspace

	mkdir -p /some/path/to/a/new/ros/toolchain
	cd /some/path/to/a/new/ros/toolchain
	
Generate the build files

	rosinstall_generator desktop_full --rosdistro hydro --deps --wet-only --tar > hydro-desktop-full-wet.rosinstall
	wstool init -j8 src hydro-desktop-full-wet.rosinstall

Resolve any dependencies

	rosdep install --from-paths src --ignore-src --rosdistro hydro -y

Build and install

	sudo src/catkin/bin/catkin_make_isolated --install

Temporarily source the new toolchain

	source /some/path/to/a/new/ros/toolchain/devel_isolated/setup.bash

You should now be able to run ROS. Test the following command

	rocscore

Install gzweb (optional)

	mkdir -p /some/path/to/a/new/workspace
	cd /some/path/to/a/new/workspace
	wget http://gazebosim.org/assets/distributions/gzweb-1.1.0.tar.bz2
	tar -xjvf gzweb-1.1.0.tar.bz2
	cd gzweb-1.1.0
	sudo npm install
	./deploy.sh -m

Checkout uav_framework into a new workspace

	mkdir -p /some/path/to/a/new/workspace
	cd /some/path/to/a/new/workspace
	git clone https://bitbucket.org/asymingt/uav_framework.git

Build the UAV framework

	cd uav_framework
	catkin_make

Add the following to ~/.bashrc 

	source /some/path/to/a/new/ros/toolchain/devel_isolated/setup.bash

Edit /usr/share/gazebo/setup.sh to point to 

	export GAZEBO_MASTER_URI=http://localhost:11345
	export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models
	export GAZEBO_MODEL_PATH=/some/path/to/a/new/workspace/uav_framework/src/uav_qrsim/resources/models
	export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-2.2:/some/path/to/a/new/workspace/uav_framework/src/uav_qrsim/resources
	export GAZEBO_PLUGIN_PATH=/usr/lib/gazebo-2.2/plugins:/some/path/to/a/new/workspace/uav_framework/devel/lib
	export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.8.0


Running instructions
=========================

This assumes that you have a network formed by the following devices
 1. bas.uas.local : The ground station used as a communication backbone
 2. fpX.uas.local : An unmanne daerial vehicle, where X is from 0 ... N
 3. gui.uas.local : A thin client for monitoring / controlling the experiment

It will be assumed that
 1. All devices are in constant communication range
 2. The device gs.uas.local has significant computational power

Run the ROS communication core on bas.uas.local

	roscore

Run the simulation core on bas.uas.local.

	rosrun uav_qrsim gzserver

Boot all fpX.uas.local and they should run the following automatically

	rosrun uav_asctec
	
Wait until the /uav topic appears on the ROS messaging backbone

	rostopic list

At this point you can run either the QT visualisation client (laptops)

	gzclient

Or, you can point a browser to the base station (mobile devices)

	http://bas.uas.local
	
QRSim may exist in two different states -- visualisation and simulation. In the visualisation state it does its best to provide an accurate visualisation of an experiment in progress. In the simulation state it acts as a full multi-platform simulator. By default, it is in the simulation state and awaiting commands from the abitrator. The moment any message is written to the /hardware topic, it immediately switches to a visualisation state.
