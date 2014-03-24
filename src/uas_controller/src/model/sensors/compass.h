#ifndef UAS_CONTROLLER_COMPASS_H
#define UAS_CONTROLLER_COMPASS_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// HAL includes
#include <uas_hal/peripheral/Magnetic.h>

// Core functionality
#include "sensor.h"

namespace uas_controller
{
	class Compass : public uas_hal::Magnetic, public Sensor
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
        void SetMeteorological(const double &temperature);

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetMagneticField();
	};
}

#endif