#ifndef UAS_CONTROLLER_SHEAR_H
#define UAS_CONTROLLER_SHEAR_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "component.h"

namespace uas_controller
{
    class Shear : public Component
    {

    private:

        // Current wind vector
        gazebo::math::Vector3 wind;
        
    	// Direction of the mean wind field
    	gazebo::math::Vector3 d20;
    	
    	// Speed of the wind in feet per second
    	double s20;

    	// Constant parameters
    	double mA;		// Minimum wind altitude
    	double z0;		// Constant (based on flight envelope)    	

    public:

    	// Default constructor
    	Shear();

        // Default constructor takes configuration + pointer to link
        void Configure(sdf::ElementPtr root);

    	// Set the speed (m) and direction (degrees) of the winf
    	void SetWind(const double &speed, const double &direction);
    	
    	// Get the wind vector based on the 
    	gazebo::math::Vector3 Update(const gazebo::physics::LinkPtr &link, const double &dt);

        // Get the wind vector based on the 
        gazebo::math::Vector3 GetVelocity();

    };
}

#endif