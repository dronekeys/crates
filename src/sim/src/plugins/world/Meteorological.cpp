// Standard data containers
#include <list>

// Rinex processing data
#include <gpstk/CommonTime.hpp>
#include <gpstk/RinexMetData.hpp>
#include <gpstk/RinexMetHeader.hpp>
#include <gpstk/RinexMetStream.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>

// Protobuf messages
#include "environment.pb.h"
#include "meteorological.pb.h"

/*
namespace gazebo
{
	class Meteorological : public WorldPlugin
	{

	private:

		// Parameters 
      	double te, pr, hu;		// Default meteorological
      	double t, h, p;			// Current meteorological
      	double rate;			// Broadcast rate

	    // Required for messaging
	    physics::WorldPtr 		worldPtr;
		transport::NodePtr 		nodePtr;
		transport::PublisherPtr pubPtr;
		ros::Timer 				timer;
		ros::NodeHandle 		rosNode;

	    // For storing RINEX meteorlogical data
	    std::list<gpstk::RinexMetData> ml;
	    std::list<gpstk::RinexMetData>::iterator mi;

      	// Message containing information
      	msgs::Meteorological msg;

        // Get the current time in a specific format
        gpstk::CommonTime GetTime(const gpstk::TimeSystem &ts)
        {
            // Get the current time tick, which is the start of the experiment plus simulated time
            gpstk::CommonTime ret = currentTime;

            // UTC -> GPS
            if (ts == gpstk::TimeSystem::GPS)
            {
                ret.addSeconds(
                    gpstk::TimeSystem::Correction(
                        gpstk::TimeSystem::UTC, gpstk::TimeSystem::GPS,        // Source & Dest
                        startTime.year, startTime.month, startTime.day         // Rough period
                    )
                );
            }

            // Set the time system
            ret.setTimeSystem(ts);

            // Return the time
            return ret;
        }

		// Send a gazebo meterological message
		void Update(const ros::TimerEvent& event)
		{
			// Try to see if we can grab some weather data from the RINEX file
			try
			{
				// Seek for the data
				while ((!ml.empty())  && (mi!= ml.end() && (*mi).time < GetTime(gpstk::TimeSystem::UTC))) 
				   mi++; 

				// Set the parent class values, so other children can access them!
				t = (*mi).data[gpstk::RinexMetHeader::TD];
				h = (*mi).data[gpstk::RinexMetHeader::HR];
				p = (*mi).data[gpstk::RinexMetHeader::PR];
			}
			catch(const std::exception& e)
			{
				ROS_WARN("Problem querying weather data: %s", e.what());
			}

			// Assemble the epoch message, with or without RINEX data
			msg.set_temperature(t);
			msg.set_pressure(p);
			msg.set_humidity(h);

			// Publish wind information to all subscribers
			pubPtr->Publish(msg);
		}

	public:

		// Default constructor
		Meteorological() : rosNode(ros::NodeHandle("meteorological")), te(273), pr(1000.0), hu(95.0), rate(1.0)
		{
			// Make sure that ROS actually started, or there will be some issues...
			if (!ros::isInitialized())
			    ROS_FATAL("A ROS node has not been initialized");
		}

		// All sensors must be configured using the current model information and the SDF
		void Load(physics::WorldPtr world, sdf::ElementPtr root)
		{
			// Save the world pointer
			worldPtr = world;

			// Parameters
			root->GetElement("rate")->GetValue()->Get(rate);
			root->GetElement("default")->GetElement("kelvin")->GetValue()->Get(te);
			root->GetElement("default")->GetElement("millibars")->GetValue()->Get(pr);
			root->GetElement("default")->GetElement("relhumidity")->GetValue()->Get(hu);

			// Will be useful for parsing the sdf
			sdf::ElementPtr el;

			// Extract any meterological data
			try
			{
				el = root->GetElement("rinex")->GetFirstElement();
				do
				{
					// Obtain the URI
					std::string uri;
					el->GetValue()->Get(uri);

					// Open meteorological data file
					gpstk::RinexMetStream rms(
						common::SystemPaths::Instance()->FindFileURI(uri).c_str()); 
					gpstk::RinexMetHeader rmh;
					gpstk::RinexMetData   rmd;

				// Let's read the header (may be skipped)
				rms >> rmh;

				// Read data into linked list
				while (rms >> rmd)
				{
				  // We need to specify that this file is in UTC
				  rmd.time.setTimeSystem(gpstk::TimeSystem::GPS);
				  ml.push_back(rmd);
				}

				}
				while(el = el->GetNextElement());

				// Set the iterator to the beginning of the linked list
				mi = ml.begin();
			}
			catch (const std::exception& e)
			{
				ROS_WARN("Could not obtain any meteorological information: %s",e.what());
			}

			// Issue a reset to prepare the data for reading
			Reset();
		}

		// All sensors must be resettable
		void Reset()
		{
			// Default meterological values
			t = te;
			h = hu;
			p = pr;

			// Reset the RINEX iterator
			mi = ml.begin();

			// START PUBLISHING DATA //////////////////////////////////////////////////////////

			// Initialize a new Gazebo transport node
			nodePtr = transport::NodePtr(new transport::Node());
			nodePtr->Init(worldPtr->GetName());
			pubPtr = nodePtr->Advertise<msgs::Meteorological>("~/meteorological");

			// CREATE A CALLBACK TIMER ///////////////////////////////////////////////////////

			// ROS timer respects gazebo
			timer = rosNode.createTimer(
				ros::Duration(1.0/rate),    	// Duration
				&Meteorological::Update,   		// Callback
				this
			);     
		}
	};

	// Resgister the plugin
	GZ_REGISTER_WORLD_PLUGIN(Meteorological);
}
*/