#ifndef UAS_HAL_NAVIGATION_H
#define UAS_HAL_NAVIGATION_H

// Local library incldues
#include <uas_hal/MsgAltitude.h>
#include <uas_hal/MsgInertial.h>
#include <uas_hal/MsgPosition.h>
#include <uas_hal/MsgMagnetic.h>
#include <uas_hal/MsgAttitude.h>
#include <uas_hal/MsgState.h>

namespace uas_hal
{
    // This is a simple filter
    class Navigation
    {

    private:

        // State estimate
        MsgState state;

    public:

        // Constructor
        Navigation();

        // Allow for periodic sensor measurements
        void Measurement(const MsgAltitude &msg);
        void Measurement(const MsgInertial &msg);
        void Measurement(const MsgPosition &msg);
        void Measurement(const MsgMagnetic &msg);
        void Measurement(const MsgAttitude &msg);

        // Get the latest state estimate
        const MsgState& GetState();
    };
}

#endif