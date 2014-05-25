#include "Orientation.h"

using namespace gazebo;

// Constructor
Orientation::Orientation() {}

// All sensors must be configured using the current model information and the SDF
bool Orientation::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
	// Backup the link
	linkPtr = link;

	// Initialise the noise distribution
	nAng = NoiseFactory::Create(root->GetElement("errors")->GetElement("orientation"));

	// Success
	return true;
}

// All sensors must be resettable
void Orientation::Reset()
{
	// Reset random number generators
	nAng->Reset();
}

// Get the current altitude
bool Orientation::GetMeasurement(double t, hal_sensor_orientation::Data& msg)
{
	// Get the quantities we want
	newAng = linkPtr->GetWorldPose().rot.GetAsEuler() + nAng->DrawVector(t);
	newTim = t;
	
	// Diff measurements to get angvel
	if (newTim - oldTim > 0)
		newVel = (newAng - oldAng) / (newTim - oldTim);

	// Package up message
	msg.t     = newTim;
	msg.roll  = newAng.x;
	msg.pitch = newAng.y;
	msg.yaw   = newAng.z;
	msg.p     = newVel.x;
	msg.q     = newVel.y;
	msg.r     = newVel.z;

	// Carry values
	oldAng = newAng;
	oldTim = newTim;

	// Success
	return true;
}
