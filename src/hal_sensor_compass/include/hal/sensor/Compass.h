#ifndef HAL_SENSOR_COMPASS_H
#define HAL_SENSOR_COMPASS_H

// Header libraries
#include <hal/sensor/Sensor.h>

// Message libraries
#include <hal_sensor_compass/Data.h>

namespace hal
{
    namespace sensor
    {
        class Compass : public Sensor<hal_sensor_compass::Data>
        {       
        public:
        	Compass(ros::NodeHandle& node);
        };
    }
}

#endif