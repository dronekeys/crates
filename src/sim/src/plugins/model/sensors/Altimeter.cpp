#include "Altimeter.h"

using namespace gazebo;

// Consutrcutor
Altimeter::Altimeter() : altOld(0), timOld(0) {}

// All sensors must be configured using the current model information and the SDF
bool Altimeter::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
	// Backup the link
	linkPtr = link;

	// Create the noise distribution
	nAlt = NoiseFactory::Create(root->GetElement("errors")->GetElement("altitude"));

	// Success!
	return true;
}

// All sensors must be resettable
void Altimeter::Reset()
{
	// Reset the noise distribution
	nAlt->Reset();

	// Reset the integrator
	altOld = 0.0;
	timOld = 0.0;
}


// Get the current altitude
bool Altimeter::GetMeasurement(double t, hal_sensor_altimeter::Data& msg)
{
	// Get the quantities we want
	altNew = linkPtr->GetWorldPose().pos.z;
	timNew = t;

	// Calculat the time since last measurement was taken
	double dt = timNew - timOld;

	// Calculate height and vertical velocity
	msg.t = timNew;
	msg.z = altNew + nAlt->DrawScalar(dt);			
	if (dt > 0)
		msg.w = (altNew - altOld) / dt;

	// Backup the altitude
	altOld = altNew;
	timOld = timNew;

	// On iteration 2+ things are fine
	return true;
}
