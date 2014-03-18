#ifndef UAS_HAL_AUV_H
#define UAS_HAL_AUV_H

// ROS includes
#include <ros/ros.h>

namespace uas_hal
{
    class HAL
    {

    protected: 

        // Is the HAL bound to 
        bool bound;

    	// Handle to the ROS node
		ros::NodeHandle node;

        // Constructor
        HAL();

    	// Perform binding to the HAL
    	void bind(const char *name);

    	// Is this HAL bounded?
    	bool isBound();

        // Get a handle to the ROS node
        ros::NodeHandle& getNodeHandle();

    public:

        // Default initialization (should be overloaded)
        void initialize(const char *name);

	};

}

#endif