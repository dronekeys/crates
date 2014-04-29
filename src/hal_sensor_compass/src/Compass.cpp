// Library headers
#include <hal/sensor/Compass.h>

using namespace hal::sensor;

Compass::Compass(ros::NodeHandle& node) : Sensor<hal_sensor_compass::Data>(node,"compass")
{

}        