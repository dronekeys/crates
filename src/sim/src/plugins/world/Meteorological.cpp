// Standard data containers
#include <list>

// UTC / GPS time management
#include <gpstk/CommonTime.hpp>
#include <gpstk/CivilTime.hpp>

// Rinex processing data
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

namespace gazebo
{
	typedef const boost::shared_ptr<const msgs::Environment> EnvironmentPtr;

	class Meteorological : public WorldPlugin
	{

	private:

		// Parameters 
		bool 						ready;			// True when UTC start has been received
      	double 						te, pr, hu;		// Default meteorological
      	double 						t, h, p;		// Current meteorological
      	double 						rate;			// Broadcast rate

	    // Required for gazebo  messaging
	    physics::WorldPtr 			worldPtr;
		transport::NodePtr 			nodePtr;
		transport::PublisherPtr 	pubPtr;
		transport::SubscriberPtr 	subPtr;

		// Required for ROS interaction
		ros::Timer 					timer;
		ros::NodeHandle 			rosNode;

      	// Message containing information
      	msgs::Meteorological 		msg;

      	// Time messages
      	gpstk::CommonTime           currentTime, startTime;
		gpstk::CivilTime 			civilTime;

	    // For storing RINEX meteorlogical data
	    std::list<gpstk::RinexMetData> ml;
	    std::list<gpstk::RinexMetData>::iterator mi;

		// Send a gazebo meterological message
		void Update(const ros::TimerEvent& event)
		{
			// If the library is not ready, then return immediately
			if (!ready)
				return;

			// Try to see if we can grab some weather data from the RINEX file
			try
			{
				// Calculate UTC -> GPS correction
				double correction = gpstk::TimeSystem::Correction(
                    gpstk::TimeSystem::UTC, gpstk::TimeSystem::GPS,
                    civilTime.year, civilTime.month, civilTime.day
                );

    			// Get the current GPS time	
	            currentTime = startTime;
				currentTime.addSeconds(worldPtr->GetSimTime().Double()); 		// + simulated time
	            currentTime.addSeconds(											// + time correction
	            	gpstk::TimeSystem::Correction(			
                  	  	gpstk::TimeSystem::UTC, gpstk::TimeSystem::GPS,
                   	 	civilTime.year, civilTime.month, civilTime.day
                   	)
                );								
	            currentTime.setTimeSystem(gpstk::TimeSystem::GPS);				// Set to GPS
			}
			catch(const std::exception& e)
			{
				ROS_ERROR("Problem converting time: %s", e.what());
			}

			// Try to see if we can grab some weather data from the RINEX file
			try
			{
				// Seek for the data
				while ((!ml.empty())  && (mi!= ml.end() && (*mi).time < currentTime)) 
				   mi++; 
			
				// Set the parent class values, so other children can access them!
				msg.set_temperature((*mi).data[gpstk::RinexMetHeader::TD]);
				msg.set_humidity((*mi).data[gpstk::RinexMetHeader::HR]);
				msg.set_pressure((*mi).data[gpstk::RinexMetHeader::PR]);

				// Publish wind information to all subscribers
				pubPtr->Publish(msg);
			}
			catch(const std::exception& e)
			{
				ROS_DEBUG("Problem querying weather data: %s", e.what());
			}
			
		}

		// This will be called whenever a new meteorlogical topic is posted
		void ReceiveEnvironment(EnvironmentPtr& env)
		{
			// We are now ready to broadcast
			ready = true;

			// Try to see if we can grab some weather data from the RINEX file
			try
			{
				// Set the time and time system
	            civilTime = gpstk::CivilTime(
	            	env->year(),
	            	env->month(),
	            	env->day(),
	            	env->hour(),
	            	env->minute(),
	            	env->second(),
	            	gpstk::TimeSystem::UTC
	            );

	            // Set the start time
	            startTime = civilTime.convertToCommonTime();
	        }
			catch(const std::exception& e)
			{
				ROS_ERROR("Problem converting time: %s", e.what());
			}
		}

	public:

		// Default constructor
		Meteorological() : rosNode(ros::NodeHandle("meteorological")), 
			ready(false), te(273), pr(1000.0), hu(95.0), rate(1.0)
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
					gpstk::RinexMetStream rms(common::SystemPaths::Instance()->FindFileURI(uri).c_str()); 
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
			}
			catch (const std::exception& e)
			{
				ROS_WARN("Could not obtain any meteorological information: %s",e.what());
			}

			// Initialize a new Gazebo transport node
			nodePtr = transport::NodePtr(new transport::Node());
			nodePtr->Init(worldPtr->GetName());

			// Advertise meterological messages
			pubPtr = nodePtr->Advertise<msgs::Meteorological>("~/meteorological");

			// Subscribe to meteorological updates
			subPtr = nodePtr->Subscribe("~/environment",&Meteorological::ReceiveEnvironment, this);

			// ROS timer respects gazebo
			if (rate > 0)
			{
				timer = rosNode.createTimer(
					ros::Duration(1.0/rate),    	// Duration
					&Meteorological::Update,   		// Callback
					this,
			        false,                                      // Oneshot
			        true                                        // Autostart
				);
			}

			// Issue a reset
			Reset();	
		}

		// All sensors must be resettable
		void Reset()
		{
			// We can only begin transmitting when the UTC start time is known
			ready = false;

			// Default meterological values
			t = te;
			h = hu;
			p = pr;

			// Reset the RINEX iterator
			mi = ml.begin();
		}
	};

	// Resgister the plugin
	GZ_REGISTER_WORLD_PLUGIN(Meteorological);
}
