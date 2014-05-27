Overview
========

CRATES stands for "Cognitive Robotics Architecture for Tightly-Coupled Experiments and Simulation". The overarching goal of the project is to create an easy-to-use architecture for writing high-level cognitive controllers for simulated robot problems, which can easily be transferred to real experiments. It achieves this by routing all communication through a hardware abstraction layer.

The general idea is that both simulated and actual hardware platforms inherit functionality from a common HAL. The HAL exposes itself to a controller over a messaging backbone. In so doing the HAL abstracts away from the detailed implementation, and presents itself as a general robot type, such as a 'quadrotor'. 

In addition to offering this abstraction service, the HAL may also include type-specific perception, navigation and low-level control algorithms. By implementing such functionality directly in the HAL, functionality is reused between experiments and simulation, and the mission-critical functions do not contend with bandwidth-intensive applications on the messaging system.

CRATES is based upon several open source and actively-developed libraries. ROS is used for the entire messaging backbone, while Gazebo is used for simulation and visualisation. In addition, the GPS toolkit is used to simulate global navigation satellite systems, while GeographicLib is used to to project between coordinate frames and calculate gravitational and magnetic fields.

Installation instructions
=========================

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

Follow the instructions to build Gazebo 3.0 from source using ODE only. If you disable building the test suite, Gazbo builds much quicker!

	http://gazebosim.org/wiki/3.0/install

Follow the instructions to build Gazebo 3.0 models

	http://gazebosim.org/wiki/3.0/install

Make sure you create this link, or gazebo wont see your models

	ln -s ~/.gazebo/models /usr/share/models

Take note that you may need need to update your OS's library search path. For example, add the following to /etc/ld.so.conf.d/gazebo.config

	/usr/local/lib/x86_64-linux-gnu
	/usr/local/lib

Then run

	sudo ldconfig

Install ROS Indigo

	sudo apt-get install ros-indigo-roscpp ros-indigo-roslaunch ros-indigo-rosbash ros-indigo-geometry-msgs ros-indigo-std-msgs ros-indigo-std-srvs ros-indigo-rostopic ros-indigo-std-rosservice

Source the new ROS installation

	source /opt/ros/indigo/setup.bash

Initialise and update ROS

	sudo rosdep init 
	rosdep update 

Checkout the CRATES framework

	cd ~/workspace
	git clone https://bitbucket.org/asymingt/crates.git

Build and install the two libraries in the ./thirdparty directory, eg. for each library 'xyz' do

	cd  ~/workspace/crates/thirdparty/xyz
	mkdir build
	cd build
	cmake ..
	make
	sudo make install

Source the new workspace ~/.bashrc 

	cd  ~/workspace/crates/src
	catkin_init_workspace

Build the CRATES framework

	cd ~/workspace/crates
	catkin_make

Source the new CRATES installation 

	source ~/crates/devel/setup.bash

You can now follow the basic usage instructions below.

Example usage instructions
==========================

The ROS messaging architecture offers broadcast-style 'topics' or request-response style 'services'. We'll use the 'roslaunch' application to launch the simulator from the command line:

	roslaunch sim sw.launch world:=worlds/hawkshead.world

The argument 'sim' is the package in CRATES containing the sw.launch file. The last argument is the world that we want to load in stead of an empty default.

If you have setup your development environment correctly, you should see a simulated version of the Royal Veterinary College Hawkshead Campus.

At the bottom of the interface you will see that the simulator provides the ability to step, pause and reset wrld time. All ROS clocks are bound to simulated time, and so any ROS timer (eg. sensor data rates, control update loops in the HAL, etc.) will abide by this simulated time.

For your convenience, time has been paused. Like any platform, the simulator offers several services on the messaging backbone. To see these, type:

	rosservice list

The list will contain the following, amongst others:

	/simulator/Delete
	/simulator/Insert
	/simulator/Noise
	/simulator/Pause
	/simulator/Reset
	/simulator/Resume
	/simulator/Seed

To resume the simulation we are going to need to call the /simulator/Resume service. Let's get a list of the arguments we'll need to send to this service:

	rosservice args /simulator/Resume

You will see that it takes none. So, go ahead and call it without arguments:

	rosservice call /simulator/Pause

Time should now be ticking in the simulation. Try playing with the Pause and Step services. Note that the Step service takes an integer number of steps as an argument. Also note that the commands all support tab-completion. 

Arguments follow JSON-style syntax:

	rosservice call /simulator/Step "num_steps: 1"

But this useful shorthand also works for simple messages

	rosservice call /simulator/Step 1

Now, add a single 'hummingbird' quadrotor model to the simulation:

	rosservice call /simulator/Insert UAV1 model://hummingbird

You should see a model appear in simulation. In addition to apparing in the simulation, the HAL inherited by the simulator has also presented itself on the messaging backbone. Use the rosservice tool to see what services it offers.

In addition to offering request-response services, the simulated entity also offers some broadcast-style messages on topics. To see what the platform offers, use the rostopic tool. 

For example, each platform has a truthful state (Truth) and an estimated version of this state (Estimate). Since we are currently in simulated mode, both of these states are observable. For example, to see the estimated state, use the following command.

	rostopic echo /hal/hummingbird/UAV1/Estimate

If nothing appears, its likely that you have the simulation paused. Remember that clocks are bound to simulated time, and if you have time paused then no callbacks will be triggered. In the very special case where you have no wind, dynamic or sensor noise, then the Estimate equals the Truth.

Finally, it is possible to launch a hardware version of an experiment using the hw.launch file using the collowing command

	roslaunch sim hw.launch world:=worlds/hawkshead.world

This command will again open an interface to a similar-looking world. However, you will notice that there are no /simulator services. This is because the simulator is listening for real (hardware) platforms on the ROS messaging backbone. If some device (a real platform) connected to ROS master server and broadcasts on some message /hal/xxx/yyy/State, then the simulator will pick up on this, and spawn a model that represents the hardware platform.