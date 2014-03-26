#ifndef UAS_CONTROLLER_GNSS_H
#define UAS_CONTROLLER_GNSS_H

// Standar dincludes
#include <string>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// HAL includes
#include <uas_hal/peripheral/Position.h>

// Make the GNSS subsystem aware of GNSS message types
#include "environment.pb.h"

// Core functionality
#include "sensor.h"

namespace uas_controller
{
    // Reference to a GPS ephemeride
    typedef const boost::shared_ptr<const uas_controller::msgs::Environment> EnvironmentPtr;

    // Basic GNSS receiver class
    class GNSS : public uas_hal::Position, public Sensor
    {

    private:

        // Pointer to the current model
        gazebo::physics::ModelPtr    modPtr;

        // GPS parameters
        bool gps_L1, gps_L2, gps_eph, gps_clk, gps_rel, gps_tro, gps_ion; 

        // Glonass parameters
        bool glo_L1, glo_L2, glo_eph, glo_clk, glo_rel, glo_tro, glo_ion; 
        
  
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
        gazebo::math::Vector3 GetPosition();

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetVelocity();

    };
}

#endif