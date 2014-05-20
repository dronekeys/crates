#ifndef SIM_SENSOR_GNSS_H
#define SIM_SENSOR_GNSS_H

// HAL functionality
#include <hal/sensor/GNSS.h>

// Basic sensor functionality
#include "Sensor.h"

namespace gazebo
{
  class GNSS : public Sensor
  {

  public:

    // All sensors must be configured using the current model information and the SDF
    bool Configure(sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_gnss::Data& msg);

  };

}

#endif