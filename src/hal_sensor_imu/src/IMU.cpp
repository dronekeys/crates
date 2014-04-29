// Library headers
#include <hal/sensor/IMU.h>

using namespace hal::sensor;

IMU::IMU(ros::NodeHandle& node) : Sensor<hal_sensor_imu::Data>(node, "imu")
{

}        