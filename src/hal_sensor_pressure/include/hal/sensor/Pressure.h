#ifndef HAL_SENSOR_PRESSURE_H
#define HAL_SENSOR_PRESSURE_H

// Header libraries
#include <hal/Sensor.h>

// Message libraries
#include <hal_sensor_pressure/Data.h>

namespace hal
{
    namespace sensor
    {
        class Pressure : public Sensor<hal_sensor_pressure::Data>
        {       
        public:
        	Pressure();
        };
    }
}

#endif