#ifndef HAL_SENSOR_ALTIMETER_H
#define HAL_SENSOR_ALTIMETER_H

// Header libraries
#include <hal/sensor/Sensor.h>

// Message libraries
#include <hal_sensor_altimeter/Data.h>

namespace hal
{
    namespace sensor
    {
        class Altimeter : public Sensor<hal_sensor_altimeter::Data>
        {       
        public:
        	Altimeter(ros::NodeHandle& node);
        };
    }
}

#endif