// Library headers
#include <hal/sensor/Energy.h>

using namespace hal::sensor;

Energy::Energy(ros::NodeHandle& node) : Sensor<hal_sensor_energy::Data>(node,"energy")
{

}        