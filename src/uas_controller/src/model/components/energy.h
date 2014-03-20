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

        // How much battery remains (amp hours)
        double remaining;

    public:

    	// Default constructor
    	Energy();

    	// Configure from plugin sdf
    	void Configure(sdf::ElementPtr root);

        // Reset the energy
        void Reset();

        // Checks whether the energy is getting low
        bool IsLow();

    	// Checks whether the energy is too low to continue flying
		bool IsCriticallyLow();

        // Predict voltage 
        double GetVoltage();

		// Update the energy 
		double Update(const double &thrust, const double &dt);

    };
}

#endif