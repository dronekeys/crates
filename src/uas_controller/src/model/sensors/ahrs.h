#ifndef UAS_CONTROLLER_AHRS_H
#define UAS_CONTROLLER_AHRS_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// HAL includes
#include <uas_hal/peripheral/Orientation.h>

// Core functionality
#include "sensor.h"

namespace uas_controller
{
    class AHRS : public uas_hal::Orientation, public Sensor
    {

    private:

        // Pointer to the current model
        gazebo::physics::ModelPtr    modPtr;

    public:

        AHRS();

        // REQUIRED METHODS

        // All sensors must be configured using the current model information and the SDF
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // All sensors must be resettable
        void Reset();

        // EXTRA METHODS

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetOrientation();
    };
}

#endif