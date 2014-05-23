#include "IMU.h"

using namespace gazebo;

// Constructor
IMU::IMU() {}

// All sensors must be configured using the current model information and the SDF
bool IMU::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
	// Backup the link
	linkPtr = link;

	// Initialise the noise distribution
	nLinAcc = NoiseFactory::Create(root->GetElement("errors")->GetElement("linacc"));
	nAngVel = NoiseFactory::Create(root->GetElement("errors")->GetElement("angvel"));

	// Succes!
	return true;
}

// All sensors must be resettable
void IMU::Reset()
{
	// Initialise the noise distribution
	nLinAcc->Reset();
	nAngVel->Reset();
}

// Get the current altitude
bool IMU::GetMeasurement(double t, hal_sensor_imu::Data& msg)
{
	// Get the quantities we want
	math::Vector3 linAcc = linkPtr->GetRelativeLinearAccel();
	math::Vector3 angVel = linkPtr->GetRelativeAngularVel();

	// Perturb acceleration
	linAcc += nLinAcc->DrawVector(t);
	angVel += nAngVel->DrawVector(t);

	// Package
	msg.t  = t;
	msg.du = linAcc.x;
	msg.dv = linAcc.y;
	msg.dw = linAcc.z;
	msg.p  = angVel.x;
	msg.q  = angVel.y;
	msg.r  = angVel.z;

	// Success!	
	return true;
}
