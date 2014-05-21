#ifndef SIM_SENSOR_GNSS_H
#define SIM_SENSOR_GNSS_H

// HAL functionality
#include <hal/sensor/GNSS.h>

// GNSS basics
#include <gpstk/WGS84Ellipsoid.hpp> // GPS ellipsoid
#include <gpstk/TropModel.hpp>      // Tropospheric model
#include <gpstk/GPSEllipsoid.hpp>   // Ellipsoid model
#include <gpstk/PRSolution.hpp>     // RAIM solver

// Basic sensor functionality
#include "Sensor.h"

// We need to know the magnetic field
#include "satellites.pb.h"
#include "meteorological.pb.h"

namespace gazebo
{
  // Convenience declarations
  typedef const boost::shared_ptr<const msgs::Satellites> SatellitesPtr;
  typedef const boost::shared_ptr<const msgs::Meterological> MeteorlogicalPtr;

  class GNSS : public Sensor
  {
  private:

    // Have we received the magnetic field?
    bool ready;

    // Satellite systems
    std::vector<gpstk::SatID::SatelliteSystem>  systems;

    // Requirements for listening for Gazbeo messages
    event::ConnectionPtr            conPtr;
    transport::NodePtr              nodePtr;
    transport::SubscriberPtr        subPtr;

    // Buffers the satellite positions locally
    msgs::Satellites                msg;

    // Used to convert from gazebo local to spherical
    gpstk::WGS84Ellipsoid           wgs84;

    // An ellipsoid for the earth
    gpstk::GPSEllipsoid             ellip;

    // RAIM solver without rotation correction (NB)
    gpstk::PRSolution               solver;

    // Tropospheric correction model
    gpstk::ZeroTropModel            tropModelZero;
    gpstk::TropModel*               tropModel;

    // Store the position and avelocity
    math::Vector3                   posNew, posOld, velNew;

    // When new environment data arrives
    void Receive(SatellitesPtr msg);

    // Solve the navigation problem
    bool Solve();

  public:

    // Constructor
    GNSS();

    // All sensors must be configured using the current model information and the SDF
    bool Configure(sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(physics::LinkPtr linkPtr, hal_sensor_gnss::Data& msg);

  };

}

#endif