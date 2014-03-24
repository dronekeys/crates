#ifndef UAS_CONTROLLER_ALTIMETER_H
#define UAS_CONTROLLER_ALTIMETER_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// HAL includes
#include <uas_hal/peripheral/Altitude.h>

// Core functionality
#include "sensor.h"

namespace uas_controller
{
	class Altimeter : public uas_hal::Altitude, public Sensor
	{

	private:

	    // Pointer to the current model
	    gazebo::physics::ModelPtr	 modPtr;

	  	// Store temp, humidity and pressure
	  	double t0, p0, h0;


	public:

		Altimeter();

        // REQUIRED METHODS

        // All sensors must be configured using the current model information and the SDF
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // All sensors must be resettable
        void Reset();

        // EXTRA METHODS

        // Set the pressure and height at ground level
        void SetMeteorological(const double &temperature, const double &pressure, const double &humidity);

        // Get the atmospheric pressure at the current altitude
        double GetPressure();
	};
}

#endif