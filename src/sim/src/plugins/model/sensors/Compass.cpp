#include "Compass.h"

using namespace gazebo;

// All sensors must be configured using the current model information and the SDF
bool Compass::Configure(sdf::ElementPtr root)
{
  return true;
}

// All sensors must be resettable
void Compass::Reset()
{

}

// Get the current altitude
bool Compass::GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_compass::Data& msg)
{
  return true;
}
