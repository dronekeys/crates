The CRATES applications requires Ubuntu, the Robotic Operating System (ROS) and Gazebo. Each of these dependencies is under active development, with its own release schedule. The installation methodology was chosen to balance ease of installation with feature support. In order for CRATES to take advantage of new Gazebo 3.0 features, like DEM terrain paging, we will compile 

Importantly, CRATES does not make use of the gazebo_ros package.

Install Ubuntu 14.04 "Trusty Tahr" as an OS (recommended) or in a Virtual Machine (not recommended -- for performance reasons)

	Visit http://releases.ubuntu.com/14.04/
	

Add the ROS PPA to your Ubuntu Sources

	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list' 
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add - 
	

Add the Gazebo PPA to your Ubuntu Sources

	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	

Update the system packages list

	sudo apt-get update

Follow the instructions to build Gazebo 3.0 from source using ODE only:

	http://gazebosim.org/wiki/3.0/install

Follow the instructions to build Gazebo 3.0 models

	http://gazebosim.org/wiki/3.0/install

Follow the instructions to build Gazebo 3.0 from source:

	ln -s ~/.gazebo/models /usr/share/models

Take note that you may need need to update your OS's library search path. For example, add the following to /etc/ld.so.conf.d/gazebo.config

	/usr/local/lib/x86_64-linux-gnu
	/usr/local/lib

Then run

	sudo ldconfig

Install basic ROS Indigo

	sudo apt-get install ros-indigo-roscpp ros-indigo-roslaunch ros-indigo-rosbash ros-indigo-geometry-msgs ros-indigo-std-msgs ros-indigo-std-srvs ros-indigo-rostopic ros-indigo-std-rosservice

Initialise and update ROS

	sudo rosdep init 
	rosdep update 

Create a new catkin workspace (for the CRATES framework)

	mkdir -p ~/crates/src
	cd ~/crates/src
	catkin_init_workspace

Checkout and build the CRATES framework.

	cd ~/crates/src
	git clone https://bitbucket.org/asymingt/crates.git
	catkin_make

Source the new workspace ~/.bashrc 

	source ~/crates/devel/setup.bash

Usage instructions
==================

