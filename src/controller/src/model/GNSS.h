#ifndef CONTROLLER_GNSS_H
#define CONTROLLER_GNSS_H

// Standar dincludes
#include <string>

// GNSS basics
#include <gpstk/GNSSconstants.hpp>  // For the speed of light
#include <gpstk/WGS84Ellipsoid.hpp> // GPS ellipsoid
#include <gpstk/TropModel.hpp>      // Tropospheric model
#include <gpstk/GPSEllipsoid.hpp>   // Ellipsoid model

// This exactly the normal PR solver code, except I have moved the earth rotation correction
// outside the solver, so that we can include it as an error source
#include "PRSolutionNoRotation.hpp" 

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "Model.h"

using namespace std;
using namespace gpstk;
using namespace uas_controller;

namespace controller
{
    // Reference to a GPS ephemeride
    typedef const boost::shared_ptr<const uas_controller::msgs::Environment> EnvironmentPtr;

    // Basic GNSS receiver class
    class GNSS : public Model
    {

    private:

        // Parameters
        std::string gps_typ, glo_typ;
        double gps_sig, gps_clk, gps_rel, gps_rot, gps_eph, gps_tro, gps_ion;
        double glo_sig, glo_clk, glo_rel, glo_rot, glo_eph, glo_tro, glo_ion;

        // Pointer to the current model
        gazebo::physics::ModelPtr       modPtr;
  
        // Used to convert from gazebo local to spherical
        WGS84Ellipsoid                  wgs84;
    
        // We will use this to model earth rotation
        GPSEllipsoid                    ellip;

        // RAIM solver without rotation correction (NB)
        PRSolutionNoRotation            solver;
     
        // Tropospheric correction model
        ZeroTropModel                   tropModelZero;
        TropModel*                      tropModel;


        // Current spherical position in gazebo world
        gazebo::math::Vector3           msPositionGlobal;

        // Current spherical / ECEF position in gpstk world
        gpstk::Position                 msPosGeodetic;
        gpstk::Position                 msPosGeocentric;
        gpstk::Position                 msPosECEF;

        // SoSlution information
        vector<SatID::SatelliteSystem>  systems;        // List of satellite systems
        vector<SatID>                   satellites;     // List of satellite vehicles
        Matrix<double>                  covariance;     // For weighting variou solutions!
        Matrix<double>                  ephemerides;    // All ephemerides
        Matrix<double>                  SVD;            // Solution vector 
        Vector<double>                  resids;         // Residual errors 
        Vector<double>                  slopes;         // Residual slopes

        // Intermediary store
        CommonTime                      currentTime;
        CommonTime                      oldTime;
        bool                            statusFix;
        bool                            statusDOP;
        gpstk::Position                 oldSolution;

    public:

        // Default constructor
        GNSS();

        // REQUIRED METHODS

        // All sensors must be configured using the current model information and the SDF
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // All sensors must be resettable
        void Reset();

        // EXTRA METHODS

        // Set the navigaiton solution from the ephemerides
        void SetNavigationSolution(EnvironmentPtr env);

        // Set the pressure and height at ground level
        bool GetStatusFix();
        bool GetStatusDOP();

        // Set the pressure and height at ground level
        int GetNumSats();

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetPosition();

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetVelocity();

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetDOP();

    };
}

#endif