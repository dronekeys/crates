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

      // When the FCS has been initialised
      bool                            ready;

      // Bitmask for data received
      uint8_t                         data;

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
          \return whether this can be interpreted as a valid state
      */
      bool GetState(hal_quadrotor::State &msg);

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

      //! Called to set the navigation origin. This allows LTP <-> WGS84 conversion
      /*!
        \param latitude latitude in decimal degrees north
        \param longitude longiude in decimal degrees east
        \param altitude meters above the WGS84 ellipsoid
      */
      void SetOrigin(double latitude, double longitude, double altitude);

    };
  }
}

#endif