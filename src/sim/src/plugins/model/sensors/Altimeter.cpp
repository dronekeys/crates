#include "Altimeter.h"

using namespace gazebo;

// Consutrcutor
Altimeter::Altimeter() : ready(false), last {}

// All sensors must be configured using the current model information and the SDF
bool Altimeter::Configure(sdf::ElementPtr root)
{
	// Creat the noise distribution
	nAlt = NoiseFactory::Create(root->GetElement("errors")->GetElement("z"));

	// Success!
	return true;
}

// All sensors must be resettable
void Altimeter::Reset()
{
	// Reset the noise distribution
	nAlt.Reset();

	// Reset readiness
	ready = false;
}


// Get the current altitude
bool Altimeter::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_altimeter::Data& msg)
{
	// Get the quantities we want
	altNew = linkPtr->GetWorldPose().pos.z;

	// Perturb 
	msg.z = altNew + nAlt.Sample(linkPtr, dt);		// Velocity
	msg.w = (altNew - altOld) / dt;					// Vertical speed

	// Backup the altitude
	altOld = altNew;

	// On iteration 2+ things are fine
	if (ready) 
		return true;

	// Indicate this is the first iteration
	ready = true;
	return false;
}
