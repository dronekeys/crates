#include "Orientation.h"

using namespace gazebo;

// Constructor
Orientation::Orientation() {}

// All sensors must be configured using the current model information and the SDF
bool Orientation::Configure(sdf::ElementPtr root)
{
	// Initialise the noise distribution
	nRotX = NoiseFactory::Create(root->GetElement("errors")->GetElement("roll"));
	nRotY = NoiseFactory::Create(root->GetElement("errors")->GetElement("pitch"));
	nRotZ = NoiseFactory::Create(root->GetElement("errors")->GetElement("yaw"));
	nAngX = NoiseFactory::Create(root->GetElement("errors")->GetElement("p"));
	nAngY = NoiseFactory::Create(root->GetElement("errors")->GetElement("q"));
	nAngZ = NoiseFactory::Create(root->GetElement("errors")->GetElement("r"));
	
	// Success
	return true;
}

// All sensors must be resettable
void Orientation::Reset()
{
	// Reset random number generators
	nRotX.Reset();
	nRotY.Reset();
	nRotZ.Reset();
	nAngX.Reset();
	nAngY.Reset();
	nAngZ.Reset();
}

// Get the current altitude
bool Orientation::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_orientation::Data& msg)
{
	// Get the quantities we want
	math::Vector3 rot = linkPtr->GetWorldPose().rot.GetAsEuler();
	math::Vector3 ang = linkPtr->GetRelativeAngularVel();

	// Perturb orientation
	msg.roll  = rot.x + nRotX.Sample(linkPtr, dt);
	msg.pitch = rot.y + nRotY.Sample(linkPtr, dt);
	msg.yaw   = rot.z + nRotZ.Sample(linkPtr, dt);

	// Perturb angular velocity
	msg.p     = ang.x + nAngX.Sample(linkPtr, dt);
	msg.q     = ang.y + nAngY.Sample(linkPtr, dt);
	msg.r     = ang.z + nAngZ.Sample(linkPtr, dt);

	// Success
	return true;
}
