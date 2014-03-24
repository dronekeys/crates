#ifndef UAS_CONTROLLER_IMU_H
#define UAS_CONTROLLER_IMU_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// HAL includes
#include <uas_hal/peripheral/Inertial.h>

// Core functionality
#include "sensor.h"

namespace uas_controller
{
	class IMU : public uas_hal::Inertial, public Sensor
	{

	private:

	    // Pointer to the current model
	    gazebo::physics::ModelPtr	 modPtr;

	    // Current temperature
	    double t;

	public:

		// Default constructor
		IMU();

        // REQUIRED METHODS

        // All sensors must be configured using the current model information and the SDF
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // All sensors must be resettable
        void Reset();

        // EXTRA METHODS

        // Set the pressure and height at ground level
        void SetMeteorological(const double &temperature);

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetAngularVelocity();

        // Set the pressure and height at ground level
        gazebo::math::Vector3 GetLinearAcceleration();

	};
}

#endif