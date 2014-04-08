#ifndef CONTROLLER_AHRS_H
#define CONTROLLER_AHRS_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "Model.h"

namespace controller
{
    class AHRS : public uas_hal::Orientation, public Model
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