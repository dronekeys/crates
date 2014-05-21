#include "Compass.h"

using namespace gazebo;

// Constructor
Compass::Compass() : ready(false) {}

// When new environment data arrives
void Compass::Receive(EnvironmentPtr msg)
{
	// Set the navigation frame magnetic field
	mag.x = msg->magnetic().x();
	mag.y = msg->magnetic().y();
	mag.z = msg->magnetic().z();

	// We have been initialised
	ready = true;
}

// All sensors must be configured using the current model information and the SDF
bool Compass::Configure(sdf::ElementPtr root)
{
	// Initialise the noise distribution
	nMagX = NoiseFactory::Create(root->GetElement("errors")->GetElement("m_x"));
	nMagY = NoiseFactory::Create(root->GetElement("errors")->GetElement("m_y"));
	nMagZ = NoiseFactory::Create(root->GetElement("errors")->GetElement("m_z"));
	
    // Initialise a node pointer
    nodePtr = transport::NodePtr(new transport::Node());
    nodePtr->Init(modPtr->GetWorld()->GetName());

    // Subscribe to messages about wind conditions
    subPtr = nodePtr->Subscribe("~/environment", &Compass::Receive, this);

    //  Create a pre-physics update call
    conPtr = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Aerodynamics::PrePhysics, this, _1));

    // Reset
    Reset();

    // Success!
	return true;
}

// All sensors must be resettable
void Compass::Reset()
{
	// Reset random number generators
	nMagX.Reset();
	nMagY.Reset();
	nMagZ.Reset();

	// Reset ready flag
	ready = false;
}

// Get the current altitude
bool Compass::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_compass::Data& msg)
{
	// Get the quantities we want
	math::Vector3 magB = linkPtr->GetWorldPose().rot.GetInverse().RotateVector(mag);

	// Perturb angular velocity
	msg.m_x  = magB.x + nMagX.Sample(linkPtr, dt);
	msg.m_y  = magB.y + nMagY.Sample(linkPtr, dt);
	msg.m_z  = magB.z + nMagZ.Sample(linkPtr, dt);

	// Success!	
	return ready;
}
