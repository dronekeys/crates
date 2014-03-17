#ifndef UAS_HAL_MAGNETIC_H
#define UAS_HAL_MAGNETIC_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>

namespace uas_hal
{
    class Magnetic : public HAL
    {
    public:

        // Connstructor
        Magnetic();

        // Initialise the Hardware Abstraction Layer
        void bind(ros::NodeHandle& nh);

    private:

        // HAL state
        bool bound;
    };
}

#endif