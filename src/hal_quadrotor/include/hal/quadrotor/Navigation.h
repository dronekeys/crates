#ifndef HAL_MODEL_QUADROTOR_NAVIGATION_H
#define HAL_MODEL_QUADROTOR_NAVIGATION_H

// Sensors
#include <hal/sensor/Altimeter.h>
#include <hal/sensor/Compass.h>
#include <hal/sensor/IMU.h>
#include <hal/sensor/GNSS.h>

// Platform state
#include <hal_model_quadrotor/State.h>

// ROS
#include <ros/ros.h>

namespace hal
{
  namespace model
  {
    // Define the gaussian types
    typedef enum 
    {
      NAVIGATION_NAIVE,
      NAVIGATION_DISABLED
    } 
    NavigationType;

    // A 6DoF kinematic model for quadrotor navigation. Uses inertial
    // measurements to update the pose of the quadrotor. Periodically
    // corrects the pose with measurements from a compass, GNSS and
    // altimeter. Source code adapted from the OpenPilot project.
    class Navigation :
      public hal::sensor::Altimeter,
      public hal::sensor::Compass,
      public hal::sensor::IMU,
      public hal::sensor::GNSS,
      public hal::sensor::Orientation
    {     

    private:
      
      /// Whether a measurement has been received
      bool bootstrapped;

      /// The estimated state
      hal_model_quadrotor state;

    public:    

      //! Initialise the navigation engine
      /*!
          \param nh the ROS node handle
          \param navigation the default navigation engine
      */
      void Init(ros::NodeHandle* nh, NavigationType navigation);
      
      //! Switch to a new navigation engine
      /*!
          \param navigation the new navigation engine
      */
      void Switch(NavigationType navigation);

      //! Get the current state
      /*!
        \param msg reset the navigation engine
      */
      void Reset();

      //! Called when new altimeter data arrives
      /*!
        \param msg the sensor data
      */
      void RcvAltimeter(const hal_sensor_altimeter::Data& msg);

      //! Called when new compass data arrives
      /*!
        \param msg the sensor data
      */
      void RcvCompass(const hal_sensor_compass::Data& msg);

      //! Called when new IMU data arrives
      /*!
        \param msg the sensor data
      */
      void RcvIMU(const hal_sensor_imu::Data& msg);

      //! Called when new GNSS data arrives
      /*!
        \param msg the sensor data
      */
      void RcvGNSS(const hal_sensor_gnss::Data& msg);

      //! Get the current state
      /*!
        \param msg state structure to be written
      */
      void GetState(hal_model_quadrotor::State& msg);

    };
  }
}

#endif