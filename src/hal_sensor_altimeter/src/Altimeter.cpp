// Library headers
#include <hal/sensor/Altimeter.h>

using namespace hal::sensor;

Altimeter::Altimeter(ros::NodeHandle& node) : Sensor<hal_sensor_altimeter::Data>(node, "altimeter")
{

}        