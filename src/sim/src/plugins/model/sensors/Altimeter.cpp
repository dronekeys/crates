#include "Altimeter.h"

using namespace gazebo;

// All sensors must be configured using the current model information and the SDF
bool Altimeter::Configure(sdf::ElementPtr root)
{
  return true;
}

// All sensors must be resettable
void Altimeter::Reset()
{

}

// Get the current altitude
bool Altimeter::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_altimeter::Data& msg)
{
  return true;
}
