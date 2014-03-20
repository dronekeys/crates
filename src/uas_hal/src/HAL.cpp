// ROS includes
#include <ros/ros.h>

// Local library incldues
#include <uas_hal/HAL.h>

using namespace uas_hal;

// Constructor
HAL::HAL(const char *name) : node(ros::NodeHandle(name))
{
    if (!ros::isInitialized())
        ROS_FATAL("A ROS node for Gazebo has not been initialized");
}

// Obtain the control
ros::NodeHandle& HAL::GetROSNode()
{
    // Return the ros node handle
    return node;
}
