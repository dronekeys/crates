#ifndef UAS_CONTROLLER_TURBULENCE_H
#define UAS_CONTROLLER_TURBULENCE_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

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

    	// Direction of the mean wind field
    	gazebo::math::Vector3 gust;

    	// Intermediary sigma and length scale
    	gazebo::math::Vector3 s, l;

    	// Used in the update() call
    	double d, h, k;
    	
    	// Speed
    	double s20;

    	// Direction
    	gazebo::math::Quaternion d20;    	

    public:

    	// Default constructor
    	Turbulence();

    	// Change the wind parameters 
    	SetParameters(const double &mA, const double &z0);

    	// Set the speed (m) and direction (degrees) of the winf
    	void SetGlobalField(const double &speed,const double &direction);
    	
    	// Get the wind vector based on the 
    	gazebo::math::Vector3 GetGlobalVelocity(const double &alt);

    };
}

#endif