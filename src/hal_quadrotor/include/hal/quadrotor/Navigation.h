#ifndef HAL_QUADROTOR_NAVIGATION_H
#define HAL_QUADROTOR_NAVIGATION_H

// Sensors
#include <hal/sensor/Altimeter.h>
#include <hal/sensor/Compass.h>
#include <hal/sensor/IMU.h>
#include <hal/sensor/GNSS.h>
#include <hal/sensor/Orientation.h>

// Platform state
#include <hal_quadrotor/State.h>

// ROS
#include <ros/ros.h>

namespace hal
{
  namespace quadrotor
  {
    // A really simple filter for fusing data
    class Navigation
    {     

    private:
      
      /// The estimated state
      hal_quadrotor::State state;

    public:    

      /// INITIALIZE AND RESET THE NAVIGATION ENGINE //////////////////

      //! Get the current state
      /*!
        \param msg reset the navigation engine
      */
      void Reset();

      /// GET AND SET THE QUADROTOR STATE //////////////////////////////

      //! Get the state estimate
      /*!
          \param state the current platform state
      */
      void GetState(hal_quadrotor::State &msg);

      //! Set the state estimate
      /*!
          \param state the current platform state
      */
      void SetState(const hal_quadrotor::State &msg);

      /// RECEIVE CALLS FOR ALL SENSOR DATA ////////////////////////////

      //! Called when new altimeter data arrives
      /*!
        \param msg the sensor data
      */
      void Process(const hal_sensor_altimeter::Data& msg);

      //! Called when new compass data arrives
      /*!
        \param msg the sensor data
      */
      void Process(const hal_sensor_compass::Data& msg);

      //! Called when new IMU data arrives
      /*!
        \param msg the sensor data
      */
      void Process(const hal_sensor_imu::Data& msg);

      //! Called when new GNSS data arrives
      /*!
        \param msg the sensor data
      */
      void Process(const hal_sensor_gnss::Data& msg);

      //! Called when new orientation data arrives
      /*!
        \param msg the sensor data
      */
      void Process(const hal_sensor_orientation::Data& msg);

    };
  }
}

#endif