// Local library incldues
#include <uas_hal/platform/UAV.h>

using namespace uas_hal;

UAV::UAV(const char *name) : 
	HAL(name),
	actAnglesHeight		(HAL::GetROSNode(), "AnglesHeight",   boost::bind(&UAV::cbAnglesHeight_goal, 	this, _1), false),
	actEmergency		(HAL::GetROSNode(), "Emergency", 	  boost::bind(&UAV::cbEmergency_goal, 		this, _1), false),
	actLand				(HAL::GetROSNode(), "Land", 		  boost::bind(&UAV::cbLand_goal, 			this, _1), false),
	actTakeoff			(HAL::GetROSNode(), "Takeoff", 		  boost::bind(&UAV::cbTakeoff_goal, 		this, _1), false),
	actVelocity			(HAL::GetROSNode(), "Velocity", 	  boost::bind(&UAV::cbVelocity_goal, 		this, _1), false),
	actVelocityHeight	(HAL::GetROSNode(), "VelocityHeight", boost::bind(&UAV::cbVelocityHeight_goal, 	this, _1), false),
	actWaypoint			(HAL::GetROSNode(), "Waypoint", 	  boost::bind(&UAV::cbWaypoint_goal, 		this, _1), false)
{
	// Periodically, a UAV may wish to broadcast its state
	pubState = GetROSNode().advertise<uas_hal::MsgState>("State", 10);

	// Periodically, a UAV may wish to broadcast information about itself
	pubInformation = GetROSNode().advertise<uas_hal::MsgInformation>("Information", 10);

	// Allow premption of the following actions
	/*
	actAnglesHeight.registerPreemptCallback(boost::bind(&UAV::cbAnglesHeight_kill, this, _1));
	actEmergency.registerPreemptCallback(boost::bind(&UAV::cbEmergency_kill, this, _1));
	actLand.registerPreemptCallback(boost::bind(&UAV::cbLand_kill, this, _1));
	actTakeoff.registerPreemptCallback(boost::bind(&UAV::cbTakeoff_kill, this, _1));
	actVelocity.registerPreemptCallback(boost::bind(&UAV::cbVelocity_kill, this, _1));
	actVelocityHeight.registerPreemptCallback(boost::bind(&UAV::cbVelocityHeight_kill, this, _1));
	actWaypoint.registerPreemptCallback(boost::bind(&UAV::cbWaypoint_kill, this, _1));
	*/

	// Set the new control goal
	ctlAnglesHeight.SetGoal(0.0,0.0,1.0,5.0);

	// Set the control timer (always happens at 50Hz, and respects simulated time)
	timCtl = GetROSNode().createTimer(
		ros::Duration(0.02), 			// Duration
		&UAV::cbAnglesHeight_prog, 		// Callback
		this);							// Handle
}

// Controller progress
void UAV::cbAnglesHeight_prog(const ros::TimerEvent& event)
{
	// Obtain the control
	ctlAnglesHeight.Update(&state, 0.02, &control);

	// Update the HAL implementation
	ReceiveControl(
		control.pitch,
		control.roll,
		control.yaw,
		control.throttle
	);
}

// Goal implementations
void UAV::cbAnglesHeight_goal(const uas_hal::AnglesHeightGoalConstPtr &goal)
{
	// Set the new control goal
	ctlAnglesHeight.SetGoal
	(
		goal->pitch,
		goal->roll,
		goal->yaw,
		goal->altitude
	);

	// Set the control timer (always happens at 50Hz, and respects simulated time)
	timCtl = GetROSNode().createTimer(
		ros::Duration(0.02), 			// Duration
		&UAV::cbAnglesHeight_prog, 		// Callback
		this);							// Handle
}


void UAV::cbEmergency_goal(const uas_hal::EmergencyGoalConstPtr &goal) {}
void UAV::cbLand_goal(const uas_hal::LandGoalConstPtr &goal) {}
void UAV::cbTakeoff_goal(const uas_hal::TakeoffGoalConstPtr &goal) {}
void UAV::cbVelocity_goal(const uas_hal::VelocityGoalConstPtr &goal) {}
void UAV::cbVelocityHeight_goal(const uas_hal::VelocityHeightGoalConstPtr &goal) {}
void UAV::cbWaypoint_goal(const uas_hal::WaypointGoalConstPtr &goal) {}

// Goal implementations
/*
void UAV::cbAnglesHeight_kill() {}
void UAV::cbEmergency_kill() {}
void UAV::cbLand_kill() {}
void UAV::cbTakeoff_kill() {}
void UAV::cbVelocity_kill() {}
void UAV::cbVelocityHeight_kill() {}
void UAV::cbWaypoint_kill() {}
*/

// Set the state of the vehicle
void UAV::SetState(
    const double &x, 	const double &y, 	 const double &z,
    const double &roll, const double &pitch, const double &yaw,
    const double &u, 	const double &v, 	 const double &w,
    const double &p, 	const double &q, 	 const double &r,
    const double &thrust, const double &energy)
{
	state.x 	 = x; 
	state.y 	 = y; 
	state.z 	 = z;
	state.roll 	 = pitch; 
	state.pitch  = roll; 
	state.yaw 	 = yaw;
	state.u 	 = u; 
	state.v 	 = v; 
	state.w 	 = v;
	state.p 	 = p; 
	state.q 	 = q; 
	state.r 	 = r;
	state.thrust = thrust; 
	state.energy = energy; 
}

// Send the current state 
void UAV::PostState()
{
	// Set the message parameters
	msgState.tick   = ros::Time::now();
	msgState.x 	 	= state.x; 
	msgState.y 	 	= state.y; 
	msgState.z 	 	= state.z;
	msgState.roll 	= state.pitch; 
	msgState.pitch  = state.roll; 
	msgState.yaw 	= state.yaw;
	msgState.u 	 	= state.u; 
	msgState.v 	 	= state.v; 
	msgState.w 	 	= state.v;
	msgState.p 	 	= state.p; 
	msgState.q 	 	= state.q; 
	msgState.r 	 	= state.r;
	msgState.thrust = state.thrust; 
	msgState.energy = state.energy; 

	// Send the message
	pubState.publish(msgState);
}

// Send some general information
void UAV::PostInformation(
    const char*     id,
    const char*     version,
    const double&   uptime)
{
	// Set the message parameters
	msgInformation.tick 	= ros::Time::now();
	msgInformation.id 		= id;
	msgInformation.version 	= version;
	msgInformation.uptime 	= uptime;

	// Send the message
	pubInformation.publish(msgInformation);
}