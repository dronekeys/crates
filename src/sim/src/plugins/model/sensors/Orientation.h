#ifndef SIM_SENSOR_ORIENTATION_H
#define SIM_SENSOR_ORIENTATION_H

// HAL functionality
#include <hal/sensor/Orientation.h>

// Basic sensor functionality
#include "Sensor.h"

namespace gazebo
{
  class Orientation : public Sensor
  {
  private:

    // Link onto which sensor is attached
    physics::LinkPtr    linkPtr;
    
    // Noise streams
    Noise               *nRot, *nAng;

  public:

    // Constructor
    Orientation();

    // All sensors must be configured using the current model information and the SDF
    bool Configure(physics::LinkPtr link, sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(double t, hal_sensor_orientation::Data& msg);

  };

}

#endif