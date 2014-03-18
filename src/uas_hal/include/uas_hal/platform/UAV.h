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

        // Setup the altitude sensor
        void initialize(const char *name);

		// Send the current state 
		void postState();

		// Send some general information
		void postInformation();

    };
}

#endif