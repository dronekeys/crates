#ifndef HAL_SENSOR_IMU_H
#define HAL_SENSOR_IMU_H

// Header libraries
#include <hal/sensor/Sensor.h>

// Message libraries
#include <hal_sensor_imu/Data.h>

namespace hal
{
    namespace sensor
    {
        class IMU : public Sensor<hal_sensor_imu::Data>
        {       
        public:
        	IMU(ros::NodeHandle& node);
        };
    }
}

#endif