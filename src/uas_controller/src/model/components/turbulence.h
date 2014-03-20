#ifndef UAS_CONTROLLER_TURBULENCE_H
#define UAS_CONTROLLER_TURBULENCE_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "component.h"

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
        void Configure(sdf::ElementPtr root, const gazebo::physics::LinkPtr &link);

        // Set the speed (m) and direction (degrees) of the winf
        void SetWind(const double &speed, const double &direction);

        // Set the speed (m) and direction (degrees) of the winf
        void Reset(const gazebo::physics::LinkPtr &link);

        // Get the wind vector based on the 
        gazebo::math::Vector3 Update(const gazebo::physics::LinkPtr &link, const double &dt);
        
        // Get the wind vector based on the 
        gazebo::math::Vector3 GetVelocity();

    };
}

#endif