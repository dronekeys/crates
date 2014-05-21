#ifndef SIM_SENSOR_IMU_H
#define SIM_SENSOR_IMU_H

// HAL functionality
#include <hal/sensor/IMU.h>

// Basic sensor functionality
#include "Sensor.h"

namespace gazebo
{
  class IMU : public Sensor
  {
  private:

    // Noise streams
    Noise* nLinAccX, nLinAccY, nLinAccZ;
    Noise* nAngVelX, nAngVelY, nAngVelZ;

  public:

    // Constructor
    IMU();

    // All sensors must be configured using the current model information and the SDF
    bool Configure(sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_imu::Data& msg);

  };

}

#endif