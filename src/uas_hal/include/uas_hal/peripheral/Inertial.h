#ifndef UAS_HAL_INERTIAL_H
#define UAS_HAL_INERTIAL_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>
#include <uas_hal/MsgInertial.h>

namespace uas_hal
{
    class Inertial : public HAL
    {

    private:

        // Message to be published
        MsgInertial     msg;

        // Publisher for the message
        ros::Publisher  pub;

    public:

        // Setup the altitude sensor
        void initialize(const char *name);

        // Send altitude immediately with current time stamp
        void post();
    };
}

#endif