#ifndef UAS_CONTROLLER_COMPASS_H
#define UAS_CONTROLLER_COMPASS_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "Model.h"

namespace uas_controller
{
	class Compass : public Model
	{

	private:

	    // Pointer to the current model
	    gazebo::physics::ModelPtr modPtr;

	    // Magnetic vector
	    gazebo::math::Vector3 mag;

	    // Current temperature
	    double t;

	public:

		Compass();

        // REQUIRED METHODS

        // All sensors must be configured using the current model information and the SDF
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // All sensors must be resettable
        void Reset();

        // EXTRA METHODS

        // Set the pressure and height at ground level
        void SetMeteorological( const double &temperature,
    		const double &Bx, const double &By, const double &Bz);

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetMagneticField();
	};
}

#endif