// Local library incldues
#include <uas_hal/platform/UAV.h>

// For managing actions
#include <actionlib/server/simple_action_server.h>

using namespace uas_hal;

UAV::UAV() : bound(false) {}

void UAV::bind(ros::NodeHandle& nh)
{
	// Initialise topics
	pub_Information 	= nh.advertise<uas_hal::Information>("Information", 10);
	pub_State 			= nh.advertise<uas_hal::State>("State", 10);

	// The basic assumption will be that, unless told otherwise, the UAV will attempt
	// to hold its position. There are five possible ways of controlling the UAV, each
	// of which is accessed through a ROS action. This will spawn a timer to perform
	// control at a fixed rate. It will be assumed that the 
	/*
	act_Waypoint(nh, "Action/Waypoint", 
		boost::bind(&UAV::Waypoint_init, this, _1), false),		// Request callback
  	act_Waypoint.registerPreemptCallback(
  		boost::bind(&UAV::Waypoint_kill, this));				// Premption callback
	*/
	// Everything is now bound
	bound = true;
}

//////////////////////
// ACTION CALLBACKS //
//////////////////////

/*
// When this is called, it means that a new waypoint must be moved to
void UAV::Waypoint_init(const uas_hal::WaypointGoalConstPtr &goal)
{
	timer = nh.createTimer(
        ros::Duration(0.02),            // 50Hz
        &platform_asctec::cb_heartbeat, // callback
        false                           // repeat indefinitely
    );
}

// When this is called, it means that a new waypoint must be moved to
void UAV::Waypoint_kill(void)
{
	// Stop 
	controlTimer.stop();

	// Mark the goal is 
	act_Waypoint.setPreempted();
}
*/

/////////////////////
// STATE CALLBACKS //
/////////////////////

bool UAV::Information(uas_hal::Information& information)
{
	if (bound)
		pub_Information.publish(information); 
	else
		ROS_WARN("UAV HAL not initialised, so publishing is not possible");
}

bool UAV::State(uas_hal::State& state)
{ 
	if (bound) 
		pub_State.publish(state);
	else
		ROS_WARN("UAV HAL not initialised, so publishing is not possible");
}
