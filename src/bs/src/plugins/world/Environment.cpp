// Required to calculate magnetic field and gravity for current location
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/GravityModel.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// GPSTK time management
#include <gpstk/CommonTime.hpp>
#include <gpstk/CivilTime.hpp>

// ROS communication subsystem (mainly for debugging)
#include <ros/ros.h>

// Protobuf messages
#include "environment.pb.h"

namespace gazebo
{
	class Environment : public WorldPlugin
	{

	private:

		// Parameters from the SDF config file
      	int ye, mo, da, ho, mi, rate; double se;

	    // Required for messaging
	    physics::WorldPtr 		worldPtr;
		transport::NodePtr 		nodePtr;
		transport::PublisherPtr pubPtr;
		ros::Timer 				timer;
		ros::NodeHandle 		rosNode;

	    // Stores the current magnetic ang gravity field
	    math::Vector3 			magnetic;
	    math::Vector3 			gravity;

        // The start time and current time, both in UTC
        gpstk::CivilTime     	startTime;

      	// Message containing information
      	msgs::Environment 		msg;

	    //  Called to update the world information
		void Update(const ros::TimerEvent& event)
		{
			msg.set_utc(startTime.convertToCommonTime().getDays());
			msg.mutable_gravity()->set_x(gravity.x);
			msg.mutable_gravity()->set_y(gravity.y);
			msg.mutable_gravity()->set_z(gravity.z);
			msg.mutable_magnetic()->set_x(magnetic.x);
			msg.mutable_magnetic()->set_y(magnetic.y);
			msg.mutable_magnetic()->set_z(magnetic.z);
			pubPtr->Publish(msg);
		}

    public:

		// Default constructor
		Environment() : rate(1.0), rosNode(ros::NodeHandle("environment"))
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

			// Get the rate and start time
			root->GetElement("rate")->GetValue()->Get(rate);
			root->GetElement("time")->GetElement("year")->GetValue()->Get(ye);
			root->GetElement("time")->GetElement("month")->GetValue()->Get(mo);
			root->GetElement("time")->GetElement("day")->GetValue()->Get(da);
			root->GetElement("time")->GetElement("hour")->GetValue()->Get(ho);
			root->GetElement("time")->GetElement("minute")->GetValue()->Get(mi);
			root->GetElement("time")->GetElement("second")->GetValue()->Get(se);
			
        	// Create a common time variable from the UTC info in the SDF data
            startTime = gpstk::CivilTime(ye,mo,da,ho,mi,se,gpstk::TimeSystem::UTC);

			// The global WGS84 position
			math::Vector3 msPositionGlobal = 
				worldPtr->GetSphericalCoordinates()->SphericalFromLocal(
					math::Vector3(0.0,0.0,0.0));

			// Magnetic vector for current location and date
			try
			{
				GeographicLib::MagneticModel mag("wmm2010");
				mag(
					ye,																// Reference year
					msPositionGlobal.x, msPositionGlobal.y, msPositionGlobal.z, 	// Reference pos
					magnetic.x, magnetic.y, magnetic.z								// Target field
				);
			}
			catch (const std::exception& e)
			{
				ROS_WARN("Could not determine magnetic field strength: %s",e.what());
			}

			// Gravitational vector for current position
			try
			{
				GeographicLib::GravityModel grav("egm96");
				grav.Gravity(
					msPositionGlobal.x, msPositionGlobal.y, msPositionGlobal.z, 	// Reference pos
					gravity.x, gravity.y, gravity.z									// Target field
				);
			}
			catch (const std::exception& e)
			{
				ROS_WARN("Could not determine gravitational field strength: %s",e.what());
			}

			// Set the gravity in the simulation
			worldPtr->GetPhysicsEngine()->SetGravity(gravity);

			// Set up advertisements
			Reset();
		}

		// Reset the publishers and subscribers
		void Reset()
		{
			// Create and initialize a new Gazebo transport node
			nodePtr = transport::NodePtr(new transport::Node());
			nodePtr->Init(worldPtr->GetName());

			// Create a publisher on the ~/wind topic
			pubPtr = nodePtr->Advertise<msgs::Environment>("~/environment");

			// ROS timer respects gazebo
			timer = rosNode.createTimer(
				ros::Duration(1.0/rate),
				&Environment::Update,
				this
			);               
		}
	};

	// Resgister the plugin
	GZ_REGISTER_WORLD_PLUGIN(Environment);
}