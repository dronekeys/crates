#include "Orientation.h"

using namespace gazebo;

// All sensors must be configured using the current model information and the SDF
bool Orientation::Configure(sdf::ElementPtr root)
{
  return true;
}

// All sensors must be resettable
void Orientation::Reset()
{

}

// Get the current altitude
bool Orientation::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_orientation::Data& msg)
{
  return true;
}
