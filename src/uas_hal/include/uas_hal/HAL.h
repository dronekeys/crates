#ifndef UAS_HAL_AUV_H
#define UAS_HAL_AUV_H

// ROS includes
#include <ros/ros.h>

// All HAL have some verson information
#include <uas_hal/Information.h>

namespace uas_hal
{
    class HAL
    {

    public:

    	// Initialise the Hardware Abstraction Layer
    	virtual void bind(ros::NodeHandle& nh) = 0;

    };

}

#endif