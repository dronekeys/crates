#include "Compass.h"

using namespace gazebo;

// Constructor
Compass::Compass() : ready(false) {}

// When new environment data arrives
void Compass::Receive(EnvironmentPtr& msg)
{
	// Set the navigation frame magnetic field
	mag.x = msg->magnetic().x();
	mag.y = msg->magnetic().y();
	mag.z = msg->magnetic().z();

	// We have been initialised
	ready = true;
}

// All sensors must be configured using the current model information and the SDF
bool Compass::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
	// Backup the link
	linkPtr = link;
	
	// Initialise the noise distribution
	nMag = NoiseFactory::Create(root->GetElement("errors")->GetElement("magnetic"));
	
    // Initialise a node pointer
    nodePtr = transport::NodePtr(new transport::Node());
    nodePtr->Init(linkPtr->GetModel()->GetWorld()->GetName());

    // Subscribe to messages about wind conditions
    subPtr = nodePtr->Subscribe("~/environment", &Compass::Receive, this);

    // Reset
    Reset();

    // Success!
	return true;
}

// All sensors must be resettable
void Compass::Reset()
{
	// Reset random number generators
	nMag->Reset();

	// Reset ready flag
	ready = false;
}

// Get the current altitude
bool Compass::GetMeasurement(double t, hal_sensor_compass::Data& msg)
{
	// Get the quantities we want
	math::Vector3 magB = linkPtr->GetWorldPose().rot.GetInverse().RotateVector(mag);

	//Error perturb
	magB += nMag->DrawVector(t);

	// Perturb angular velocity
	msg.t  = t;
	msg.x  = magB.x;
	msg.y  = magB.y;
	msg.z  = magB.z;

	// Success!	
	return ready;
}
