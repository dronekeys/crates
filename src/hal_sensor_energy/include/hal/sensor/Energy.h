#ifndef HAL_SENSOR_ENERGY_H
#define HAL_SENSOR_ENERGY_H

// Header libraries
#include <hal/sensor/Sensor.h>

// Message libraries
#include <hal_sensor_energy/Data.h>

namespace hal
{
    namespace sensor
    {
        class Energy : public Sensor<hal_sensor_energy::Data>
        {       
        public:
        	Energy(ros::NodeHandle& node);
        };
    }
}

#endif