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

	sudo apt-get install git

Checkout the CRATES framework

	cd ~/workspace
	git clone https://bitbucket.org/asymingt/crates.git

Build/install the ROS, gazebo and gpstk libraries using the scripts

	cd  ~/workspace/crates/thirdparty
	./install_sysdeps.sh
	./install_ros.sh
	./install_gazebo.sh
	./install_gpstk.sh

Initialise your catkin workspace

	cd  ~/workspace/crates/src
	catkin_init_workspace

Build the CRATES framework

	cd ~/workspace/crates
	catkin_make

Source the new CRATES installation 

	source ~/crates/devel/setup.bash

Optional: if you plan to interact with the simulator using MATLAB, please also install the ROS IO bridge for MATLAB for your architecture:

https://www.mathworks.co.uk/hardware-support/robot-operating-system.html

You can now follow the basic usage instructions below.

Example usage instructions
==========================

The ROS messaging architecture offers broadcast-style 'topics' or request-response style 'services'. We'll use the 'roslaunch' application to launch the simulator from the command line:

	roslaunch sim sw.launch

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

	rostopic echo /hal/UAV1/Estimate

If nothing appears, its likely that you have the simulation paused. Remember that clocks are bound to simulated time, and if you have time paused then no callbacks will be triggered. In the very special case where you have no wind, dynamic or sensor noise, then the Estimate equals the Truth.

Finally, it is possible to launch a hardware version of an experiment using the hw.launch file using the collowing command

	roslaunch sim hw.launch

This command will again open an interface to a similar-looking world. However, you will notice that there are no /simulator services. This is because the simulator is listening for real (hardware) platforms on the ROS messaging backbone. If some device (a real platform) connected to ROS master server and broadcasts on some message /hal/xxx/yyy/State, then the simulator will pick up on this, and spawn a model that represents the hardware platform.

Common issues
=============
Periodically, the following message is received on starting the simulator

	terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >'what():  remote_endpoint: Transport endpoint is not connected
	Aborted (core dumped)
  
It is caused by roslaunch firing up the GUI before the simulation server has loaded. Here's a workaround.

	roslaunch sim sw.launch gui:=false
	roslaunch sim gui.launch
 
Note that you can debug any runtime issues with the simulator in debug mode

	roslaunch sim sw.launch gui:=false bin:=server_debug

 