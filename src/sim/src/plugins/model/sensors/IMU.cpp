#include "IMU.h"

using namespace gazebo;

// All sensors must be configured using the current model information and the SDF
bool IMU::Configure(sdf::ElementPtr root)
{
	return true;
}

// All sensors must be resettable
void IMU::Reset()
{

}

// Get the current altitude
bool IMU::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_imu::Data& msg)
{
	return true;
}
