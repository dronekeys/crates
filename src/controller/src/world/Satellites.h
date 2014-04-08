#ifndef CONTROLLER_SATELLITES_H
#define CONTROLLER_SATELLITES_H

// Boost includes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Class for storing "broadcast-type" ephemerides
#include <gpstk/TropModel.hpp>
#include <gpstk/Position.hpp>
#include <gpstk/SP3EphemerisStore.hpp>
#include <gpstk/GPSEphemerisStore.hpp>
#include <gpstk/GloEphemerisStore.hpp>
#include <gpstk/IonexStore.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Messages
#include "environment.pb.h"
#include "meteorological.pb.h"
#include "satellites.pb.h"

// Core functionality
#include "World.h"

namespace controller
{
	/* This class opens broadcast and final GPS and Glonass emphemerides files
	   and publishes the satellite positions to /global/satellites based on the
	   current simulation time. To prevent unnecessary calculation at the GNSS
	   receiver, this class also calculates the per-satellite ionospheric and
	   tropospheric path delays, as well as the visiblity.                     */
	class Satellites : public World
	{

	private:

		// BASIC PARAMETERS ////////////////////////////////////////////////////

		double                              rate, mine;

		// MDATA STORAGE MECHANISMS ////////////////////////////////////////////

	    // For storing GPS and Glonass ephemerides
	    gpstk::GPSEphemerisStore    		gpsEphemerides;
	    gpstk::GloEphemerisStore    		gloEphemerides;
	    gpstk::SP3EphemerisStore    		sp3Ephemerides;
	    gpstk::IonexStore           		tecStore;

 		// Goad and Goodman (1974) troppspheric model
	    gpstk::GGTropModel                 	tropModel;
		gpstk::Triple 						ionoModel;

		// Position of the world origin in ECEF
		gpstk::Position 					originECEF;

	    // Locally store temperature, pressure and humidity (troposhperic model)
	    double 								temperature, pressure, humidity;

	    // MESSAGING RELATED VARIABLES /////////////////////////////////////////

	    // Required for messaging
	    gazebo::physics::WorldPtr 			worldPtr;
		gazebo::transport::NodePtr 			nodePtr;
		gazebo::transport::PublisherPtr 	pubPtr;
		gazebo::transport::SubscriberPtr 	subMetPtr, subWorPtr;
		ros::Timer 							timer;

      	// Message containing information
      	msgs::Satellites 					msg;

      	// CALLED PERIODICALLY TO UPDATE THE SATELLITE INFORMATION /////////////

	    //  Called to update the world information
	    void Update(const ros::TimerEvent& event);

	public:

		// Constructor
		Satellites();

        // All sensors must be configured using the current world information
        void Configure(sdf::ElementPtr root, gazebo::physics::WorldPtr model);

        // All sensors must be resettable
        void Reset();

	};
}

#endif