#ifndef UAS_CONTROLLER_COMPASS_H
#define UAS_CONTROLLER_COMPASS_H

// Required for time 
#include <gpstk/CommonTime.hpp>
#include <gpstk/CivilTime.hpp>
#include <gpstk/Position.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "World.h"

namespace uas_controller
{
	class Environment : public World
	{

	private:

		// Parameters from the SDF config file
      	int ye,mo,da,ho,mi,rate; double s;

	    // Required for messaging
	    gazebo::physics::WorldPtr 		worldPtr;
		gazebo::transport::NodePtr 		nodePtr;
		gazebo::transport::PublisherPtr pubPtr;
		ros::Timer 						timer;

	    // Stores the current magnetic and gravitational field
	    gazebo::math::Vector3 magnetic;
	    gazebo::math::Vector3 gravity;
		gazebo::math::Vector3 home;

      	// Message containing information
      	msgs::Environment msg;

	    //  Called to update the world information
	    void Update(const ros::TimerEvent& event);

	public:

		// Constructor
		Environment();

        // All sensors must be configured using the current model information and the SDF
        void Configure(sdf::ElementPtr root, gazebo::physics::WorldPtr model);

        // All sensors must be resettable
        void Reset();
	};
}

#endif