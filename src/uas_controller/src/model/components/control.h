#ifndef UAS_CONTROLLER_CONTROL_H
#define UAS_CONTROLLER_CONTROL_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

// Core functionality
#include "component.h"

namespace uas_controller
{
    class Control : public Component
    {
    public:

    	// Control is always stored
    	double pitch;
    	double roll;
    	double thrust;
    	double yaw;
    	double voltage;

    	// Default constructor takes configuration + pointer to link
    	Configure(sdf::ElementPtr _sdf);
		
		// Set the control in standard scientific units
        SetSI(
        	const double &pitch, 
        	const double &roll,
        	const double &thrust,
        	const double &yaw
		);

        // Set the voltage
		SetVoltage(const double &voltage);

    };
}

#endif