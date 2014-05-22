#include "IMU.h"

using namespace gazebo;

// Constructor
IMU::IMU() {}

// All sensors must be configured using the current model information and the SDF
bool IMU::Configure(physics::LinkPtr linkPtr, sdf::ElementPtr root)
{
	// Backup the link
	link = linkPtr;

	// Initialise the noise distribution
	nLinAccX = NoiseFactory::Create(root->GetElement("errors")->GetElement("a_x"));
	nLinAccY = NoiseFactory::Create(root->GetElement("errors")->GetElement("a_y"));
	nLinAccZ = NoiseFactory::Create(root->GetElement("errors")->GetElement("a_z"));
	nAngVelX = NoiseFactory::Create(root->GetElement("errors")->GetElement("w_x"));
	nAngVelY = NoiseFactory::Create(root->GetElement("errors")->GetElement("w_y"));
	nAngVelZ = NoiseFactory::Create(root->GetElement("errors")->GetElement("w_z"));

	// Succes!
	return true;
}

// All sensors must be resettable
void IMU::Reset()
{
	// Initialise the noise distribution
	nLinAccX.Reset();
	nLinAccY.Reset();
	nLinAccZ.Reset();
	nAngVelX.Reset();
	nAngVelY.Reset();
	nAngVelZ.Reset();
}

// Get the current altitude
bool IMU::GetMeasurement(double t, hal_sensor_imu::Data& msg)
{
	// Get the quantities we want
	math::Vector3 linAcc = linkPtr->GetRelativeLinearAcc();
	math::Vector3 angVel = linkPtr->GetRelativeAngularVel();

	// Perturb acceleration
	msg.t  = t;
	msg.du = linAcc.x + nLinAccX.Sample(linkPtr, dt);
	msg.dv = linAcc.y + nLinAccY.Sample(linkPtr, dt);
	msg.dw = linAcc.z + nLinAccZ.Sample(linkPtr, dt);
	msg.p  = angVel.x + nAngVelX.Sample(linkPtr, dt);
	msg.q  = angVel.y + nAngVelY.Sample(linkPtr, dt);
	msg.r  = angVel.z + nAngVelZ.Sample(linkPtr, dt);

	// Success!	
	return true;
}
