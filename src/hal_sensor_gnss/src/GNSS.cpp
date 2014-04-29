// Library headers
#include <hal/sensor/GNSS.h>

using namespace hal::sensor;

GNSS::GNSS(ros::NodeHandle& node) : Sensor<hal_sensor_gnss::Data>(node, "gnss")
{

}        