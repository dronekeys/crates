#ifndef UAS_CONTROLLER_DYNAMICS_H
#define UAS_CONTROLLER_DYNAMICS_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "component.h"

namespace uas_controller
{
    class Dynamics : public Component
    {

    private:

	    // Constant model parameters
	    double _LOW_THROTT, _MAX_ANGVEL, _G,  _MASS_KG;         // Platform config
	    double _pq0, _pq1, _pq2, _r0, _r1;                      // Rotational params
	    double _Cth0, _Cth1, _Cth2, _Cvb0, _Cvb1, _tau0, _tau1; // Thrust params
	    double _kuv, _kw;                                       // Drag parameters	    

        // Updating in each loop iteration
        double thrust, dFth, tau;

        // Intermediary elements
        gazebo::math::Vector3 n_rot, b_lin_vel, b_ang_vel;
        gazebo::math::Vector3 torq, forc, drag;

    public:

        // Default constructor
        Dynamics();

    	//  Configure
    	void Configure(sdf::ElementPtr root);

        // Reset the component
        void Reset();

        // Update the component
        void Update(
            const gazebo::physics::LinkPtr& link,               // Pointer to physics element       
            const gazebo::math::Vector3& wind,                  // Wind force                   
            const double& pitch,                                // RC pitch
            const double& roll,                                 // RC roll
            const double& throttle,                             // RC throttle
            const double& yaw,                                  // RC yaw
            const double& voltage,                              // RC voltage
            const double& dt);                                  // Time
    };
}

#endif