#ifndef DRONEKEY_WIRELESS_H
#define DRONEKEY_WIRELESS_H

//Wireless ROS Service!!!
#include <sim/Wireless.h>
#include <hal_quadrotor/Packet.h>

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
		 * Ros Node Reference!
		 */
		boost::shared_ptr<ros::NodeHandle>    gRosNode;

		/**
		 * World Reference
		 */
		gazebo::physics::WorldPtr gWorld;
	
		/**
		 * Store All wireless Models
		 */
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
		bool isWireless(std::string &serviceURI);
	public:

		/**
		 * Set the private world
		 */
		void SetWireless(gazebo::physics::WorldPtr &world, boost::shared_ptr<ros::NodeHandle> rosNode);

		/**
		 * Send the data to correct nodes!!
		 * @return Send Successfull or Not.
		 */
		bool Send(std::string &IP, std::string &msg);

		/**
		 * Constructor
		 */
		Wireless();

	};
}

#endif