#ifndef CONTROLLER_ALTIMETER_H
#define CONTROLLER_ALTIMETER_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "Model.h"

namespace controller
{
	class Altimeter : public Model
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

        // Get the current altitude
        double GetAltitude();

	};
}

#endif