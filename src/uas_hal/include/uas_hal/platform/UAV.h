#ifndef UAS_HAL_UAV_H
#define UAS_HAL_UAV_H

// ROS includes
#include <uas_hal/HAL.h>

// For timers
#include <ros/ros.h>

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

// Include the UAV PID controllers
#include <uas_hal/controller/AnglesHeight.h>

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

        // Support the following actions
        actionlib::SimpleActionServer<uas_hal::AnglesHeightAction>      actAnglesHeight;
        actionlib::SimpleActionServer<uas_hal::EmergencyAction>         actEmergency;
        actionlib::SimpleActionServer<uas_hal::LandAction>              actLand;
        actionlib::SimpleActionServer<uas_hal::TakeoffAction>           actTakeoff;
        actionlib::SimpleActionServer<uas_hal::VelocityAction>          actVelocity;
        actionlib::SimpleActionServer<uas_hal::VelocityHeightAction>    actVelocityHeight;
        actionlib::SimpleActionServer<uas_hal::WaypointAction>          actWaypoint;

        // This actions control
        ros::Timer      timCtl;

        // Maintain the UAV state and last control
        State           state;
        Control         control;

        // A controller
        AnglesHeight    ctlAnglesHeight;

        // Goal implementations
        void cbAnglesHeight_goal(const uas_hal::AnglesHeightGoalConstPtr &goal);
        void cbEmergency_goal(const uas_hal::EmergencyGoalConstPtr &goal);
        void cbLand_goal(const uas_hal::LandGoalConstPtr &goal);
        void cbTakeoff_goal(const uas_hal::TakeoffGoalConstPtr &goal);
        void cbVelocity_goal(const uas_hal::VelocityGoalConstPtr &goal);
        void cbVelocityHeight_goal(const uas_hal::VelocityHeightGoalConstPtr &goal);
        void cbWaypoint_goal(const uas_hal::WaypointGoalConstPtr &goal);

        // Controller callback
        void cbAnglesHeight_prog(const ros::TimerEvent& event);

        // Prempt implementations
        /*
        void cbAnglesHeight_kill();
        void cbEmergency_kill();
        void cbLand_kill();
        void cbTakeoff_kill();
        void cbVelocity_kill();
        void cbVelocityHeight_kill();
        void cbWaypoint_kill();
        */

    public:

        // Setup the altitude sensor
        UAV(const char *name);

		// Send the current state 
		void PostState();

		// Send some general information
		void PostInformation(
            const char*     id,
            const char*     version,
            const double&   uptime
            );

        // Set the state of the vehicle
        void SetState(
            const double &x,      const double &y,     const double &z,
            const double &roll,   const double &pitch, const double &yaw,
            const double &u,      const double &v,     const double &w,
            const double &p,      const double &q,     const double &r,
            const double &thrust, const double &energy);

        // Derived classes must implement this function
        virtual void ReceiveControl(
            const double &pitch,
            const double &roll, 
            const double &yaw, 
            const double &throttle) = 0;

    };
}

#endif