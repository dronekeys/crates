#ifndef UAS_CONTROLLER_TURBULENCE_H
#define UAS_CONTROLLER_TURBULENCE_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

// Core functionality
#include "component.h"

// Basic constants 
#define METERS_TO_FEET 		3.2808399
#define FEET_TO_METERS 		0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000
#define MATH_PI				3.14159265359

namespace uas_controller
{
    class Turbulence : public Component
    {

    private:

        // CBoostrapping parameters
        int     bsiter;
        double  bstime;

    	// Direction of the mean wind field
    	gazebo::math::Vector3 wind;

    	// Intermediary variables
    	gazebo::math::Vector3 s, l;
    	double d, h, k;
    	
    	// Speed and direction 
    	double s20;
    	gazebo::math::Quaternion d20;    	

    public:

        // Constructor
        Turbulence();

        // Default constructor takes configuration + pointer to link
        void Configure(sdf::ElementPtr root);

        // Set the speed (m) and direction (degrees) of the winf
        void SetWind(const double &speed, const double &direction);

        // Set the speed (m) and direction (degrees) of the winf
        void Reset(const double &alt, const double &speed);

        // Get the wind vector based on the 
        gazebo::math::Vector3 Update(const double &alt, const double &speed, const double &dt);
        
        // Get the wind vector based on the 
        gazebo::math::Vector3 GetVelocity();

    };
}

#endif