#ifndef HAL_SENSOR_GNSS_H
#define HAL_SENSOR_GNSS_H

// Header libraries
#include <hal/Sensor.h>

// Message libraries
#include <hal_sensor_gnss/Data.h>

namespace hal
{
    namespace sensor
    {
        class GNSS : public Sensor<hal_sensor_gnss::Data>
        {       
        public:
        	GNSS();
        };
    }
}

#endif