#include <hal/quadrotor/Actuation.h>

using namespace hal::quadrotor;

void Actuation::Init(ros::NodeHandle* nh, ControllerType controller)
{
	// Create services
	srvAnglesHeight 	= nh->advertiseService("controller/AnglesHeight", &Actuation::RcvAnglesHeight, this);
    srvEmergency 		= nh->advertiseService("controller/Emergency", &Actuation::RcvEmergency, this);
    srvHover 			= nh->advertiseService("controller/Hover", &Actuation::RcvHover, this);
    srvLand 			= nh->advertiseService("controller/Land", &Actuation::RcvLand, this);                 
    srvTakeoff 			= nh->advertiseService("controller/Takeoff", &Actuation::RcvTakeoff, this);
    srvVelocity 		= nh->advertiseService("controller/Velocity", &Actuation::RcvVelocity, this);
    srvVelocityHeight 	= nh->advertiseService("controller/VelocityHeight", &Actuation::RcvVelocityHeight, this);
    srvWaypoint 		= nh->advertiseService("controller/Waypoint", &Actuation::RcvWaypoint, this);

    // Set the starting controller
    Switch(controller);
}

void Actuation::Switch(ControllerType controller)
{
	current = controller;
}

bool Actuation::GetControl(const hal_quadrotor::State &state, 
	double dt, hal_quadrotor::Control &control)
{
	Controller* ptr;
	switch (current)
	{
		case CONTROLLER_ANGLESHEIGHT: 	ptr = (Controller*) &cAnglesHeight;		break;
		case CONTROLLER_EMERGENCY: 		ptr = (Controller*) &cEmergency;		break;
		case CONTROLLER_HOVER: 			ptr = (Controller*) &cHover;			break;
		case CONTROLLER_IDLE: 			ptr = (Controller*) &cIdle;				break;
		case CONTROLLER_LAND: 			ptr = (Controller*) &cLand;				break;
		case CONTROLLER_TAKEOFF: 		ptr = (Controller*) &cTakeoff;			break;
		case CONTROLLER_VELOCITYHEIGHT: ptr = (Controller*) &cVelocityHeight;	break;
		case CONTROLLER_VELOCITY: 		ptr = (Controller*) &cVelocity;			break;
		case CONTROLLER_WAYPOINT: 		ptr = (Controller*) &cWaypoint;			break;
		default:		
			return false;
	}

	// Check to see if the controller has reached its goal
	if (ptr->HasGoalBeenReached())
	{
		// At the end of takeoff / motion control, always switch to hover
		switch (current)
		{
		case CONTROLLER_TAKEOFF:
			current = CONTROLLER_HOVER;
			ptr = (Controller*) &cIdle;
			break;

		case CONTROLLER_LAND:
			current = CONTROLLER_IDLE;
			ptr = (Controller*) &cIdle;
			break;
		}
	}

	// Get some control from the controller
	return ptr->Update(state,dt,control);
}

bool Actuation::RcvAnglesHeight(
    hal_quadrotor::AnglesHeight::Request  &req, 
    hal_quadrotor::AnglesHeight::Response &res)
{
	switch (current)
	{
	case CONTROLLER_HOVER:
	case CONTROLLER_ANGLESHEIGHT: 
	case CONTROLLER_VELOCITYHEIGHT: 
	case CONTROLLER_VELOCITY: 
	case CONTROLLER_WAYPOINT:
		cAnglesHeight.SetGoal(req, res);
		if (res.success)
			Switch(CONTROLLER_ANGLESHEIGHT);
		return true;
	}
    res.success = false;
    res.status  = "Controller switch denied from current state";
	return true;
}

bool Actuation::RcvEmergency(
    hal_quadrotor::Emergency::Request  &req, 
    hal_quadrotor::Emergency::Response &res)
{
	cEmergency.SetGoal(req, res);
	if (res.success)
		Switch(CONTROLLER_EMERGENCY);
	return true;
}

bool Actuation::RcvHover(
    hal_quadrotor::Hover::Request  &req, 
    hal_quadrotor::Hover::Response &res)
{
	switch (current)
	{
	case CONTROLLER_HOVER:
	case CONTROLLER_ANGLESHEIGHT: 
	case CONTROLLER_VELOCITYHEIGHT: 
	case CONTROLLER_VELOCITY: 
	case CONTROLLER_WAYPOINT:
		cHover.SetGoal(req, res);
		if (res.success)
			Switch(CONTROLLER_HOVER);
		return true;
	}
    res.success = false;
    res.status  = "Controller switch denied from current state";
	return true;
}

bool Actuation::RcvLand(
    hal_quadrotor::Land::Request  &req, 
    hal_quadrotor::Land::Response &res)
{
	switch (current)
	{
	case CONTROLLER_HOVER:
	case CONTROLLER_ANGLESHEIGHT: 
	case CONTROLLER_VELOCITYHEIGHT: 
	case CONTROLLER_VELOCITY: 
	case CONTROLLER_WAYPOINT:
		cLand.SetGoal(req, res);
		if (res.success)
			Switch(CONTROLLER_LAND);
		return true;
	}
    res.success = false;
    res.status  = "Controller switch denied from current state";
	return true;
}

bool Actuation::RcvTakeoff(
    hal_quadrotor::Takeoff::Request  &req, 
    hal_quadrotor::Takeoff::Response &res)
{
	switch (current)
	{
	case CONTROLLER_IDLE:
		cTakeoff.SetGoal(req, res);
		if (res.success)
			Switch(CONTROLLER_TAKEOFF);
		return true;
	}
    res.success = false;
    res.status  = "Controller switch denied from current state";
	return true;
}

bool Actuation::RcvVelocity(
    hal_quadrotor::Velocity::Request  &req, 
    hal_quadrotor::Velocity::Response &res)
{
	switch (current)
	{
	case CONTROLLER_HOVER:
	case CONTROLLER_ANGLESHEIGHT: 
	case CONTROLLER_VELOCITYHEIGHT: 
	case CONTROLLER_VELOCITY: 
	case CONTROLLER_WAYPOINT:
		cVelocity.SetGoal(req, res);
		if (res.success)
			Switch(CONTROLLER_VELOCITY);
		return true;
	}
    res.success = false;
    res.status  = "Controller switch denied from current state";
	return true;
}

bool Actuation::RcvVelocityHeight(
    hal_quadrotor::VelocityHeight::Request  &req, 
    hal_quadrotor::VelocityHeight::Response &res)
{
	switch (current)
	{
	case CONTROLLER_HOVER:
	case CONTROLLER_ANGLESHEIGHT: 
	case CONTROLLER_VELOCITYHEIGHT: 
	case CONTROLLER_VELOCITY: 
	case CONTROLLER_WAYPOINT:
		cVelocityHeight.SetGoal(req, res);
		if (res.success)
			Switch(CONTROLLER_VELOCITYHEIGHT);
		return true;
	}
    res.success = false;
    res.status  = "Controller switch denied from current state";
	return true;
}

bool Actuation::RcvWaypoint(
    hal_quadrotor::Waypoint::Request  &req, 
    hal_quadrotor::Waypoint::Response &res)
{
	switch (current)
	{
	case CONTROLLER_HOVER:
	case CONTROLLER_ANGLESHEIGHT: 
	case CONTROLLER_VELOCITYHEIGHT: 
	case CONTROLLER_VELOCITY: 
	case CONTROLLER_WAYPOINT:
		cWaypoint.SetGoal(req, res);
		if (res.success)
			Switch(CONTROLLER_WAYPOINT);
		return true;
	}
    res.success = false;
    res.status  = "Controller switch denied from current state";
	return true;
}