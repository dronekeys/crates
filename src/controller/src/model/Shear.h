#ifndef UAS_CONTROLLER_SHEAR_H
#define UAS_CONTROLLER_SHEAR_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "Model.h"

namespace uas_controller
{
    class Shear : public Model
    {

    private:

        // Pointer to the model object
        gazebo::physics::ModelPtr  modPtr;

        // Input parameters from SDF
        double speed, dir, mA, z0, kuv, kw;

        // Internal parameters
        gazebo::math::Quaternion q;
    	gazebo::math::Vector3 d20, wind, drag;
        double mass, s20, a;

    public:

    	// Default constructor
    	Shear();

        // REQUIRED METHODS

        //  Configure the propulsion engine
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // Update the model based on the time step (and internal control)
        void Update(const double &dt);

        // Reset the propulsion engine
        void Reset();

        // EXTRA METHODS

    	// Set the speed (m) and direction (degrees) of the winf
    	void SetWind(const double& speed, const double& direction);

    };
}

#endif