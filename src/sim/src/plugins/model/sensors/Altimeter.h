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
  private:

    // Link onto which sensor is attached
    physics::LinkPtr  link;

    // Current and old altitude readings
    double            altNew, altOld, timNew, timOld;

    // Noise distributions
    Noise*            nAlt;

  public:

    // Constructor
    Altimeter();

    // All sensors must be configured using the current model information and the SDF
    bool Configure(physics::LinkPtr linkPtr, sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(double t, hal_sensor_altimeter::Data& msg);

  };

}

#endif