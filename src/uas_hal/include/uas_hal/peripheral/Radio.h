#ifndef UAS_HAL_RADIO_H
#define UAS_HAL_RADIO_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>

namespace uas_hal
{
    class Radio : public HAL
    {
    public:

    	// Connstructor
		Radio();

    	// Initialise the Hardware Abstraction Layer
    	void bind(ros::NodeHandle& nh);

    private:

    	// HAL state
    	bool bound;
    };

}

#endif