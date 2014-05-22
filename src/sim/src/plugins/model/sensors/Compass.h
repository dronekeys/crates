#ifndef SIM_SENSOR_COMPASS_H
#define SIM_SENSOR_COMPASS_H

// HAL functionality
#include <hal/sensor/Compass.h>

// Basic sensor functionality
#include "Sensor.h"

// We need to know the magnetic field
#include "environment.pb.h"

namespace gazebo
{
  // Environment messages
  typedef const boost::shared_ptr<const msgs::Environment> EnvironmentPtr;

  class Compass : public Sensor
  {
  private:

    // Link onto which sensor is attached
    physics::LinkPtr              link;

    // Requirements for listening for Gazbeo messages
    event::ConnectionPtr          conPtr;
    transport::NodePtr            nodePtr;
    transport::SubscriberPtr      subPtr;

    // Have we received the magnetic field?
    bool                          ready;

    // The magnetic field
    math::Vector3                 mag;

    // Noise streams
    Noise*                        nMagX, nMagY, nMagZ;

    // When new environment data arrives
    void Receive(EnvironmentPtr msg);

  public:

    // Constructor
    Compass();

    // All sensors must be configured using the current model information and the SDF
    bool Configure(physics::LinkPtr linkPtr, sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(double t, hal_sensor_compass::Data& msg);

  };

}

#endif