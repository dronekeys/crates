#ifndef UAS_HAL_POSITION_H
#define UAS_HAL_POSITION_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>

namespace uas_hal
{
    class Position : public HAL
    {
    public:

        // Connstructor
        Position();

        // Initialise the Hardware Abstraction Layer
        void bind(ros::NodeHandle& nh);

    private:

        // HAL state
        bool bound;
    };
}

#endif