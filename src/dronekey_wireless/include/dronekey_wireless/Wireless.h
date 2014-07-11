#ifndef DRONEKEY_WIRELESS_H
#define DRONEKEY_WIRELESS_H

//Wireless ROS Service!!!
#include <sim/Wireless.h>

//Include ROS
#include <ros/ros.h>
//Include Gazebo Physics
#include <gazebo/physics/physics.hh>

#include <sstream>
#include <string>
#include <map>

namespace dronekey {
	class Wireless {
	private:
		
		/**
		 * World Reference
		 */
		gazebo::physics::WorldPtr gWorld;
	
		std::map<std::string, std::string> wirelessModels;
		/**
		 * [areaNodes description]
		 * @return [description]
		 */
		int areaNodes();

		/**
		 * Check if model has a wireless service!!
		 * @param  modelName Model ID
		 * @return           whether it has a wireless or not
		 */
		bool isWireless(std::string &modelName);
	public:

		/**
		 * Set the private world
		 */
		void SetWorld(gazebo::physics::WorldPtr &world);

		/**
		 * Send the data to correct nodes!!
		 * @return Send Successfull or Not.
		 */
		bool Send();

		/**
		 * Constructor
		 */
		Wireless();

	};
}

#endif