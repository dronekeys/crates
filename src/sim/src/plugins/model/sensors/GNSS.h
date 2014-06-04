#ifndef SIM_SENSOR_GNSS_H
#define SIM_SENSOR_GNSS_H

// HAL functionality
#include <hal_sensor_gnss/GNSS.h>

// For converting Gazebo <-> ECEF coordinates
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

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
  typedef const boost::shared_ptr<const msgs::Satellites>     SatellitesPtr;
  typedef std::vector<gpstk::SatID::SatelliteSystem>          SatelliteSystemVec;

  class GNSS : public Sensor
  {
  private:

    // Link onto which sensor is attached
    physics::LinkPtr                linkPtr;

    // Have we received satellite data
    bool                            ready;

    // Requirements for listening for Gazbeo messages
    event::ConnectionPtr            conPtr;
    transport::NodePtr              nodePtr;
    transport::SubscriberPtr        subPtr;
    
    // Gazebo messages
    msgs::Satellites                sat;

    // Solver parameters
    bool                            _enabled;
    int                             _maxIterations;
    double                          _minError;
    double                          _minElevation;
    bool                            _gpsUse, _gpsEph, _gpsIon, _gpsClk, _gpsTro;
    bool                            _gloUse, _gloEph, _gloIon, _gloClk, _gloTro;

    // Internal variables used for navigation
    gpstk::CommonTime               currentTime;
    gpstk::WGS84Ellipsoid           wgs84;
    gpstk::GPSEllipsoid             ellip;
    gpstk::PRSolution               solver;
    gpstk::ZeroTropModel            tropModelZero;
    gpstk::TropModel*               tropModel;
    SatelliteSystemVec              systems;
    std::vector<gpstk::SatID>       satellites;     // List of satellite vehicles
    gpstk::Matrix<double>           covariance;     // For weighting variou solutions!
    gpstk::Matrix<double>           ephemerides;    // All ephemerides
    gpstk::Matrix<double>           SVD;            // Solution vector 
    gpstk::Vector<double>           resids;         // Residual errors 
    gpstk::Vector<double>           slopes;         // Residual slopes

    // Receiver noise
    Noise                           *nReceiver;

    // Store the position and velocity
    math::Vector3                   posNew, posOld, velNew;

    // For storing time (needed to calcualte velocity)
    double                          timOld, timNew;

    // The actual solution
    hal_sensor_gnss::Data           soln;

    // When new satellite data arrives
    void Receive(SatellitesPtr& msg);


  public:

    // Constructor
    GNSS();

    // All sensors must be configured using the current model information and the SDF
    bool Configure(physics::LinkPtr link, sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    bool GetMeasurement(double t, hal_sensor_gnss::Data& msg);

  };

}

#endif