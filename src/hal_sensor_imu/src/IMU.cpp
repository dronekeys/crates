// Library headers
#include <hal/sensor/IMU.h>

using namespace hal::sensor;

IMU::IMU() : Sensor<hal_sensor_imu::Data>("sensors/inertial")
{

}        