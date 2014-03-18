// ROS includes
#include <ros/ros.h>

// Local library incldues
#include <uas_hal/HAL.h>

using namespace uas_hal;

// Constructor
HAL::HAL() : bound(false) {}

// Check to see if this HAL has been bound to ROS
bool HAL::isBound() 
{
	return this->bound;
}

// Obtain the control
void HAL::bind(const char *name)
{
	// Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
		ROS_FATAL("A ROS node for Gazebo has not been initialized");
    else
    {
        if (bound)
            ROS_WARN("HAL is already bound. Ignoring rebind.");
        else
        {
            // Create a new ROS node with this name
        	this->node  = ros::NodeHandle(name); 

            // Bind the HAL to the ros node
            this->bound = true;
        }
    }
}

// Obtain the control
void HAL::initialize(const char *name)
{
    // Default initialize does nothing
    bind(name);
}

// Obtain the control
ros::NodeHandle& HAL::getNodeHandle()
{
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!this->bound)
        ROS_FATAL("You cannot request a node handle until binding is complete");

    // Return the ros node handle
    return this->node;
}