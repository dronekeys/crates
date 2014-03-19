#ifndef UAS_CONTROLLER_ENERGY_H
#define UAS_CONTROLLER_ENERGY_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

// Core functionality
#include "component.h"

namespace uas_controller
{
    class Energy : public Component
    {

    private:
	
    	// Parameters
    	double tot, rem, cb, cf, kill, warn, th;

		// Watt-hours remaining
		double remaining;

    public:

    	// Default constructor
    	Energy();

    	// Configure from plugin sdf
    	bool Configure();

    	// Checks whether the energy is too low to continue flying
		bool IsCriticallyLow();

		// Update the energy 
		void Update(const double &thrust, const double &dt);

		// The amount by which to reduce thrust to offer a genlt landing
		double GetThrustReduction();

		// Predict voltage 
		double GetVoltage();

    };
}

#endif