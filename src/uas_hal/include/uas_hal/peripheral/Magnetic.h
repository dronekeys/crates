#ifndef UAS_HAL_MAGNETIC_H
#define UAS_HAL_MAGNETIC_H

// ROS includes
#include <ros/ros.h>

// Base HAL type
#include <uas_hal/HAL.h>
#include <uas_hal/MsgMagnetic.h>

namespace uas_hal
{
    class Magnetic : public HAL
    {

    private:

        // Message to be published
        MsgMagnetic     msg;

        // Publisher for the message
        ros::Publisher  pub;

    public:

        // Setup the altitude sensor
        void initialize(const char *name);
        
        // Send altitude immediately with current time stamp
        void post(
            const double &x,    // Body-frame magnetic field X (Gauss)
            const double &y,    // Body-frame Magnetic field Y (Gauss)
            const double &z     // Body-frame Magnetic field Z (Gauss)
        );
    };
}

#endif