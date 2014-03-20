#ifndef UAS_HAL_ALTITUDE_H
#define UAS_HAL_ALTITUDE_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>
#include <uas_hal/MsgAltitude.h>

namespace uas_hal
{
    class Altitude : public HAL
    {

    private:

        // Message to be published
        MsgAltitude     msg;

        // Publisher for the message
        ros::Publisher  pub;

    public:

        // Setup the altitude sensor
        Altitude(const char *name);
        
        // Send altitude immediately with current time stamp
        void Post(const double &height, const double &speed);
    };
}

#endif