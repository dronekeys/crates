#ifndef UAS_HAL_UAV_H
#define UAS_HAL_UAV_H

// ROS includes
#include <uas_hal/HAL.h>

// This package's messages
#include <uas_hal/Information.h>
#include <uas_hal/State.h>

// This package's actions
//#include <uas_hal/ControlWaypointAction.h>

namespace uas_hal
{
	/* 	The UAV interface provides an abstract mechanism from which to control a UAV */
    class UAV : public HAL
    {

    public:

    	// Connstructor
		UAV();

    	// All HALs must be bound properly to the node
    	void bind(ros::NodeHandle& nh);

		// Child class offers the following services
		// void Waypoint(const uas_hal::WaypointGoalConstPtr &goal);

		// Child class publishes the following information
		bool Information(uas_hal::Information& information);
		bool State(uas_hal::State& state);

    private:

    	// Have we bound yet? 
    	bool bound;

		// Services
		ros::Publisher pub_Information;
		ros::Publisher pub_State;

		// Actions
		// actionlib::SimpleActionServer<uas_hal::WaypointAction> act_Waypoint;

        // Timer used to issue control actions
        // ros::Timer controlTimer;

    };
}

#endif