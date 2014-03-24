#ifndef UAS_CONTROLLER_GNSS_H
#define UAS_CONTROLLER_GNSS_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// HAL includes
#include <uas_hal/peripheral/Position.h>

// Core functionality
#include "sensor.h"

namespace uas_controller
{
    class GNSS : public uas_hal::Position, public Sensor
    {

    private:

        // Pointer to the current model
        gazebo::physics::ModelPtr    modPtr;

    public:

        // Default constructor
        GNSS();

        // REQUIRED METHODS

        // All sensors must be configured using the current model information and the SDF
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // All sensors must be resettable
        void Reset();

        // EXTRA METHODS

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetPosition();

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetVelocity();

    };
}

#endif