#include "GNSS.h"

using namespace gazebo;

// All sensors must be configured using the current model information and the SDF
bool GNSS::Configure(sdf::ElementPtr root)
{
	return true;
}

// All sensors must be resettable
void GNSS::Reset()
{

}

// Get the current altitude
bool GNSS::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_gnss::Data& msg)
{
	return true;
}
