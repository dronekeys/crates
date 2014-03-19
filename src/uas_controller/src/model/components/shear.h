#ifndef UAS_CONTROLLER_SHEAR_H
#define UAS_CONTROLLER_SHEAR_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

// Basic constants 
#define METERS_TO_FEET 		3.2808399
#define FEET_TO_METERS 		0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000

namespace uas_controller
{
    class Shear : public Component
    {

    private:

    	// Direction of the mean wind field
    	gazebo::math::Vector3 d20;
    	
    	// Speed of the wind in feet per second
    	double s20;

    	// Constant parameters
    	double  _mA;		// Minimum wind altitude
    	double  _z0;		// Constant (based on flight envelope)    	

    public:

    	// Default constructor
    	Shear();

    	// Change the wind parameters 
    	SetParameters(const double &mA, const double &z0);

    	// Set the speed (m) and direction (degrees) of the winf
    	void SetGlobalField(const double &speed,const double &direction);
    	
    	// Get the wind vector based on the 
    	gazebo::math::Vector3 GetGlobalVelocity(const double &alt);

    };
}

#endif