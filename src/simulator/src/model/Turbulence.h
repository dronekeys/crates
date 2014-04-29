#ifndef CONTROLLER_TURBULENCE_H
#define CONTROLLER_TURBULENCE_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Core functionality
#include "Model.h"

namespace controller
{
    class Turbulence : public Model
    {

    private:

        // Pointer to the model object
        gazebo::physics::ModelPtr  modPtr;

        // Parameters from SDF
        int     bsiter;
        double  bstime, speed, dir, kuv, kw;

    	// Used internally
        gazebo::math::Vector3 wind, s, l, drag;
        gazebo::math::Quaternion d20, q;
    	double d, a, h, k;
    	double s20;

    public:

        // Constructor
        Turbulence();

        // REQUIRED METHODS

        //  Configure the propulsion engine
        void Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model);

        // Update the model based on the time step (and internal control)
        void Update(const double &dt);

        // Reset the propulsion engine
        void Reset();

    };
}

#endif