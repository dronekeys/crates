#ifndef CONTROLLER_METEOROLOGICAL_H
#define CONTROLLER_METEOROLOGICAL_H

// Standard data containers
#include <list>

// GPStk includes
#include <gpstk/RinexMetData.hpp>

// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "World.h"

namespace controller
{
	class Meteorological : public World
	{

	private:

		// Parameters 
      	double ws, wd;			// Wind speed and direction
      	double te, pr, hu;		// Default meteorological
      	double t, h, p;			// Current meteorological
      	double rate;			// Broadcast rate

	    // Required for messaging
	    gazebo::physics::WorldPtr 		worldPtr;
		gazebo::transport::NodePtr 		nodePtr;
		gazebo::transport::PublisherPtr pubPtr;
		ros::Timer 						timer;

	    // For storing RINEX meteorlogical data
	    std::list<gpstk::RinexMetData> ml;
	    std::list<gpstk::RinexMetData>::iterator mi;

      	// Message containing information
      	msgs::Meteorological msg;

	    //  Called to update the world information
	    void Update(const ros::TimerEvent& event);

	public:

		// Constructor
		Meteorological();

        // All sensors must be configured using the current model information and the SDF
        void Configure(sdf::ElementPtr root, gazebo::physics::WorldPtr model);

        // All sensors must be resettable
        void Reset();

	};
}

#endif