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
	nRot = NoiseFactory::Create(root->GetElement("errors")->GetElement("rot"));
	nAng = NoiseFactory::Create(root->GetElement("errors")->GetElement("ang"));
	
	// Success
	return true;
}

// All sensors must be resettable
void Orientation::Reset()
{
	// Reset random number generators
	nRot->Reset();
	nAng->Reset();
}

// Get the current altitude
bool Orientation::GetMeasurement(double t, hal_sensor_orientation::Data& msg)
{
	// Get the quantities we want
	math::Vector3 rot = linkPtr->GetWorldPose().rot.GetAsEuler();
	math::Vector3 ang = linkPtr->GetRelativeAngularVel();

	// Perturb measuremnet
	rot += nRot->DrawVector(t);
	ang += nAng->DrawVector(t);

	// Package up message
	msg.t     = t;
	msg.roll  = rot.x;
	msg.pitch = rot.y;
	msg.yaw   = rot.z;
	msg.p     = ang.x;
	msg.q     = ang.y;
	msg.r     = ang.z;

	// Success
	return true;
}
