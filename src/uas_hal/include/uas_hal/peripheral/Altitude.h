#ifndef UAS_HAL_ALTITUDE_H
#define UAS_HAL_ALTITUDE_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>

namespace uas_hal
{
    class Altitude : public HAL
    {
    public:

        // Connstructor
        Altitude();

        // Initialise the Hardware Abstraction Layer
        void bind(ros::NodeHandle& nh);

    private:

        // HAL state
        bool bound;
    };
}

#endif