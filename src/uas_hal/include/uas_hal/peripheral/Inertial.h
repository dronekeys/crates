#ifndef UAS_HAL_INERTIAL_H
#define UAS_HAL_INERTIAL_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>

namespace uas_hal
{
    class Inertial : public HAL
    {
    public:

        // Connstructor
        Inertial();

        // Initialise the Hardware Abstraction Layer
        void bind(ros::NodeHandle& nh);

    private:

        // HAL state
        bool bound;
    };
}

#endif