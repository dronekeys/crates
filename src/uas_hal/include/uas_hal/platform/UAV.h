#ifndef UAS_HAL_UAV_H
#define UAS_HAL_UAV_H

// ROS includes
#include <uas_hal/HAL.h>

// This package's messages
#include <uas_hal/MsgInformation.h>
#include <uas_hal/MsgState.h>

// This package's actions
//#include <uas_hal/ControlWaypointAction.h>

namespace uas_hal
{
    // Basic control
    typedef struct
    {
        double p;
        double r;
        double t;
        double y;
    } 
    Control;

	/* 	The UAV interface provides an abstract mechanism from which to control a UAV */
    class UAV : public HAL
    {

    private:

        // Message to be published
        MsgState    	msgState;
        MsgInformation  msgInformation;

        // Publisher for the message
        ros::Publisher  pubState;
        ros::Publisher  pubInformation;

    public:

        // Derived classes must implement this function
        virtual void HalReceiveControl(const Control &ctl) = 0;

        // Setup the altitude sensor
        void HalInit(const char *name);

		// Send the current state 
		void HalBroadcastState(
            const double &px, const double &py, const double &pz,
            const double &rx, const double &ry, const double &rz,
            const double &vx, const double &vy, const double &vz,
            const double &ax, const double &ay, const double &az,
            const double &thrust, const double &energy
        );

		// Send some general information
		void HalBroadcastInformation(
            const char*     id,
            const char*     version,
            const double&   uptime
            );

    };
}

#endif