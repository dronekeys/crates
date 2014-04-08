// Required to calculate magnetic field and gravity for current location
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/GravityModel.hpp>

//  Boost includes
#include "Environment.h"

using namespace controller;

// Default constructor
Environment::Environment() : World("environment"),
	rate(1.0) {}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void Environment::Configure(sdf::ElementPtr root, gazebo::physics::WorldPtr world)
{
	// Save the world pointer
	worldPtr = world;

	// Rate at which /world messages are posted
	rate = GetSDFDouble(root,"rate",rate);

	// GET THE GAZEBO COORDINATES ///////////////////////////////////////////////////////////

	// The global WGS84 position
	gazebo::math::Vector3 msPositionGlobal = 
		worldPtr->GetSphericalCoordinates()->SphericalFromLocal(
			gazebo::math::Vector3(0.0,0.0,0.0));

	// SET THE HOME POSITION ////////////////////////////////////////////////////////////////

	// Convert to a GPStk position
	gpstk::Position originPosGeodetic(
		msPositionGlobal.x, msPositionGlobal.y, msPositionGlobal.z, gpstk::Position::Geodetic, &wgs84);

	// Convert to a geocentric position
	gpstk::Position originPosGeocentric = originPosGeodetic.transformTo(gpstk::Position::Geocentric);
	
	// Get the cartesian ECEF position
	gpstk::Position originPosECEF = originPosGeodetic.asECEF();

	// GET THE MAGNETIC VECTOR //////////////////////////////////////////////////////////////
	
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

	// GET THE GRAVITATIONAL VECTOR //////////////////////////////////////////////////////////
	
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

	// SET THE SIMULATED GRAVITY /////////////////////////////////////////////////////////////

	// Set the gravity in the simulation
	worldPtr->GetPhysicsEngine()->SetGravity(gravity);

	// RESET THE MODULE //////////////////////////////////////////////////////////////////////

	Reset();
}

// Reset the publishers and subscribers
void Environment::Reset()
{
	// Create and initialize a new Gazebo transport node
	nodePtr = gazebo::transport::NodePtr(new gazebo::transport::Node());
	nodePtr->Init(worldPtr->GetName());

	// Create a publisher on the ~/wind topic
	pubPtr = nodePtr->Advertise<msgs::Environment>("~/global/environment");

	// ROS timer respects gazebo
	timer = rosNode.createTimer(
		ros::Duration(1.0/rate),    // Duration
		&Environment::Update,    			// Callback
		this
	);               
}

// Send a gazebo environment message
void Environment::Update(const ros::TimerEvent& event)
{
	// Assemble the epoch message
	msg.mutable_gravity()->set_x(gravity.x);
	msg.mutable_gravity()->set_y(gravity.y);
	msg.mutable_gravity()->set_z(gravity.z);
	msg.mutable_magnetic()->set_x(magnetic.x);
	msg.mutable_magnetic()->set_y(magnetic.y);
	msg.mutable_magnetic()->set_z(magnetic.z);

	// Publish wind information to all subscribers
	pubPtr->Publish(msg);
}