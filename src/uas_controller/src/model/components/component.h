#ifndef UAS_CONTROLLER_COMPONENT_H
#define UAS_CONTROLLER_COMPONENT_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

namespace uas_controller
{
    class Component
    {

    protected:

    	// Default constructor takes
    	virtual Configure(sdf::ElementPtr _sdf) = 0;

    };
}

#endif