#ifndef UAS_HAL_UAV_H
#define UAS_HAL_UAV_H

// ROS includes
#include <uas_hal/HAL.h>

// For managing actions
#include <actionlib/server/simple_action_server.h>

// This package's messages
#include <uas_hal/MsgInformation.h>
#include <uas_hal/MsgState.h>

// This package's actions
#include <uas_hal/AnglesHeightAction.h>
#include <uas_hal/EmergencyAction.h>
#include <uas_hal/LandAction.h>
#include <uas_hal/TakeoffAction.h>
#include <uas_hal/VelocityAction.h>
#include <uas_hal/VelocityHeightAction.h>
#include <uas_hal/WaypointAction.h>

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

        // Support the following actions
        actionlib::SimpleActionServer<uas_hal::AnglesHeightAction>      actAnglesHeight;
        /*
        actionlib::SimpleActionServer<uas_hal::EmergencyAction>         actAnglesEmergency;
        actionlib::SimpleActionServer<uas_hal::LandAction>              actLand;
        actionlib::SimpleActionServer<uas_hal::TakeoffAction>           actTakeoff;
        actionlib::SimpleActionServer<uas_hal::VelocityAction>          actVelocity;
        actionlib::SimpleActionServer<uas_hal::VelocityHeightAction>    actVelocityHeight;
        actionlib::SimpleActionServer<uas_hal::WaypointAction>          actWaypoint;
        */

        // Callback
        void cbAnglesHeight(const uas_hal::AnglesHeightGoalConstPtr &goal);

    public:

        // Setup the altitude sensor
        UAV(const char *name);

		// Send the current state 
		void PostState(
            const double &px, const double &py, const double &pz,
            const double &rx, const double &ry, const double &rz,
            const double &vx, const double &vy, const double &vz,
            const double &ax, const double &ay, const double &az,
            const double &thrust, const double &energy
        );

		// Send some general information
		void PostInformation(
            const char*     id,
            const char*     version,
            const double&   uptime
            );

        // Derived classes must implement this function
        virtual void ReceiveControl(
            const double &pitch,
            const double &roll,
            const double &throttle,
            const double &yaw) = 0;

    };
}

#endif