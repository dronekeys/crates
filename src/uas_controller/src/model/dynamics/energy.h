#ifndef UAS_CONTROLLER_ENERGY_H
#define UAS_CONTROLLER_ENERGY_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

// Core functionality
#include "dynamics.h"

namespace uas_controller
{
    class Energy : public Dynamics
    {

    private:

        // Pointer to the model object
        gazebo::physics::ModelPtr  modPtr;
	
    	// Parameters
    	double tot, rem, cb, cf, kill, warn, th;

        // How much battery remains (amp hours)
        double remaining, throttle;

    public:

    	// Default constructor
    	Energy();

        // REQUIRED METHODS

        //  Configure the propulsion engine
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // Update the model based on the time step (and internal control)
        void Update(const double &dt);

        // Reset the propulsion engine
        void Reset();

        // EXTRA METHODS

        // Set the throttle value
        void SetThrottle(const double &th);

        // Predict voltage 
        double GetVoltage();

    };
}

#endif