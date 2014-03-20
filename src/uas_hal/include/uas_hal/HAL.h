#ifndef UAS_HAL_AUV_H
#define UAS_HAL_AUV_H

// ROS includes
#include <ros/ros.h>

namespace uas_hal
{
    class HAL
    {

    private:

        // Handle to the ROS node
        ros::NodeHandle node;

    protected: 

        // Get a handle to the ROS node
        ros::NodeHandle& GetROSNode();

    public:

        // Constructor
        HAL(const char *name);

	};

}

#endif