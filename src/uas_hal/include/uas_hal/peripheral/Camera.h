#ifndef UAS_HAL_CAMERA_H
#define UAS_HAL_CAMERA_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>

namespace uas_hal
{
    class Camera : public HAL
    {
    public:

        // Connstructor
        Camera();

        // Initialise the Hardware Abstraction Layer
        void bind(ros::NodeHandle& nh);

    private:

        // HAL state
        bool bound;
    };
}

#endif