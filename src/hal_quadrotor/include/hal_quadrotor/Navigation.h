#ifndef HAL_QUADROTOR_NAVIGATION_H
#define HAL_QUADROTOR_NAVIGATION_H

// For converting Gazebo <-> ECEF coordinates
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

// Sensors
#include <hal_sensor_altimeter/Altimeter.h>
#include <hal_sensor_compass/Compass.h>
#include <hal_sensor_imu/IMU.h>
#include <hal_sensor_gnss/GNSS.h>
#include <hal_sensor_orientation/Orientation.h>

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

      // For oordinat conversions
      GeographicLib::Geocentric       wgs84_ecef;
      GeographicLib::LocalCartesian   wgs84_enu;
    
      /// The estimated state
      hal_quadrotor::State            state;

    public:    

      /// Constructor
      Navigation();

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

      //! Set the home position (origin)
      /*!
          \param latitude the home latitude
          \param longitude the home longitude
          \param altitude the home altitude
      */
      void SetHome(double latitude, double longitude, double altitude);

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