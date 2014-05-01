#ifndef HAL_SENSOR_ENERGY_H
#define HAL_SENSOR_ENERGY_H

// Header libraries
#include <hal/sensor/ENERGY.h>

// Message libraries
#include <hal_sensor_compass/Data.h>

namespace hal
{
    namespace sensor
    {
        class Compass : public ENERGY<hal_energy_compass::Data>
        {       
        public:
        	Compass(ros::NodeHandle& node);
        };
    }
}

#endif