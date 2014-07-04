#ifndef DRONKEY_NAVIGATE
#define DRONKEY_NAVIGATE

#include <list>
#include <ros/ros.h>
#include <hal_quadrotor/control/Waypoint.h>
#include <hal_quadrotor/control/Hover.h>
#include <hal_quadrotor/State.h>

namespace dronkey {
	class Navigate {
	private:
		/**
		 * e
		 */
		std::list<hal_quadrotor::Waypoint> movement;

		/**
		 * 
		 */
		std::list<hal_quadrotor::Waypoint>::iterator movement_iterator;

		/**
		 * 
		 */
		ros::Subscriber quadState;

		/**
		 * [StateCallback description]
		 * @param msg [description]
		 */
		void StateCallback(const hal_quadrotor::State::ConstPtr& msg);
	
	public:

		Navigate(ros::NodeHandle &n);

		~Navigate();
	};
}

#endif