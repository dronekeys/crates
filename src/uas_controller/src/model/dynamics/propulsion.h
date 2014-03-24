#ifndef UAS_CONTROLLER_PROPULSION_H
#define UAS_CONTROLLER_PROPULSION_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "dynamics.h"

namespace uas_controller
{
    class Propulsion : public Dynamics
    {

    private:

        // Pointer to the model object
        gazebo::physics::ModelPtr  modPtr;

        // Initial model pose
        gazebo::math::Pose         pose;

        // Control parameters
        double srs, sps, sys, sts, svs;
        double srl, spl, syl, stl, svl;
        double sru, spu, syu, stu, svu;

	    // Dynamics parameters
	    double _LOW_THROTT, _MAX_ANGVEL, _G,  _MASS_KG;         // Platform config
	    double _pq0, _pq1, _pq2, _r0, _r1;                      // Rotational params
	    double _Cth0, _Cth1, _Cth2, _Cvb0, _Cvb1, _tau0, _tau1; // Thrust params
	    double _kuv, _kw;                                       // Drag parameters	    

        // Current control parameters
        double roll, pitch, yaw, throttle, voltage; 
        double mass, hover, thrust;
        gazebo::math::Vector3 drag;

        // Used internally in the dynamics update
        double dFth, tau;
        gazebo::math::Quaternion    q;
        gazebo::math::Vector3       o, torque, force;

        // Are the motors currently animated?
        bool motors;

        // Turn the motor animation on or off
        void AnimateMotors(bool enabled);

    public:

        // Default constructor
        Propulsion();

        // REQUIRED METHODS

    	//  Configure the propulsion engine
    	void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // Update the model based on the time step (and internal control)
        void Update(const double &dt);

        // Reset the propulsion engine
        void Reset();

        // EXTRA METHODS

        // Reset the internal control
        void SetControl(const double &r,const double &p,const double &y,const double &t,const double &v);

        // Get the current thurst force
        double GetThrust();

    };
}

#endif