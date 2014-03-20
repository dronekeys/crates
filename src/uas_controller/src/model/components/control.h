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

    private:

        // Scaling parameters
        double sp, sr, sy, st, sv;

    public:
        
        // Control, in SI units
        double pitch;
        double roll;
        double yaw;
        double throttle;
        double voltage;

        // Default constructor takes configuration + pointer to link
        Control();

    	// Default constructor takes configuration + pointer to link
    	void Configure(sdf::ElementPtr root);
		
        // Returns control scaled to RC values
        double GetScaledPitch();
        double GetScaledRoll();
        double GetScaledYaw();
        double GetScaledThrottle();
        double GetScaledVoltage();
    };
}

#endif