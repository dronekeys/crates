#ifndef SIM_SENSOR_ALTIMETER_H
#define SIM_SENSOR_ALTIMETER_H

// HAL functionality
#include <hal/sensor/Altimeter.h>

// Basic sensor functionality
#include "Sensor.h"

namespace gazebo
{
  class Altimeter : public Sensor
  {

  public:

    // All sensors must be configured using the current model information and the SDF
    bool Configure(sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_altimeter::Data& msg);

  };

}

#endif