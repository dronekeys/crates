#ifndef SIM_SENSOR_IMU_H
#define SIM_SENSOR_IMU_H

// HAL functionality
#include <hal_sensor_imu/IMU.h>

// Basic sensor functionality
#include "Sensor.h"

namespace gazebo
{
  class IMU : public Sensor
  {
  private:
    
    // Link onto which sensor is attached
    physics::LinkPtr  linkPtr;

    // Noise streams
    Noise             *nLinAcc, *nAngVel;

  public:

    // Constructor
    IMU();

    // All sensors must be configured using the current model information and the SDF
    bool Configure(physics::LinkPtr link, sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(double t, hal_sensor_imu::Data& msg);

  };

}

#endif