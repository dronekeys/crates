// Library headers
#include <hal/sensor/Altimeter.h>

using namespace hal::sensor;

Altimeter::Altimeter() : Sensor<hal_sensor_altimeter::Data>("sensors/altimeter")
{

}        