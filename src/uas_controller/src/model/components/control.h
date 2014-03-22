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

        // Scales, upper and lower limits
        double srs, sps, sys, sts, svs;
        double srl, spl, syl, stl, svl;
        double sru, spu, syu, stu, svu;

    public:
        
        // Control, in SI units
        double roll;
        double pitch;
        double yaw;
        double throttle;
        double voltage;

        // Default constructor takes configuration + pointer to link
        Control();

    	// Default constructor takes configuration + pointer to link
    	void Configure(sdf::ElementPtr root);
		
        // Returns control scaled to RC values
        double GetScaledRoll();
        double GetScaledPitch();
        double GetScaledYaw();
        double GetScaledThrottle();
        double GetScaledVoltage();
    };
}

#endif